/*
 * remote_control - Remote control car server
 *
 * TCP server on port 5103.
 * Controls motor (PWM8/PWM9 via RZ7899-MS), steering servo (PWM10),
 * and light (PWM11). Sends battery voltage telemetry every second.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <poll.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <pthread.h>
#include "ml307c.h"
#include "audio.h"
#include "log.h"


/* ======================== Configuration ======================== */

#define TCP_PORT                5103

/* 舵机标准信号参数 (50Hz, 20ms周期) */
#define SERVO_PWM_FREQ_HZ      50      /* Servo PWM frequency */
#define SERVO_PWM_PERIOD_NS    (1000000000LL / SERVO_PWM_FREQ_HZ)  /* 20000000 ns = 20ms */
#define SERVO_MIN_US           500     /* 0.5ms */
#define SERVO_CENTER_US        1500    /* 1.5ms (中间位置/停车) */
#define SERVO_MAX_US           2500    /* 2.5ms */

/* 电机PWM参数 (内置电调模式) */
#define MOTOR_PWM_FREQ_HZ      1000    /* Motor PWM frequency */
#define MOTOR_PWM_PERIOD_NS    (1000000000LL / MOTOR_PWM_FREQ_HZ)   /* 1000000 ns = 1ms */
#define MOTOR_MIN_DUTY         80      /* 电机最小启动占空比 */
#define MOTOR_REV_MAX_DUTY     150     /* 倒车最大占空比 (前进为512) */
#define MOTOR_DEAD_ZONE        10      /* 电机死区范围 */

/* 灯光PWM参数 (内置MOS管控制模式) */
#define LIGHT_PWM_FREQ_HZ      1000    /* Light PWM frequency */
#define LIGHT_PWM_PERIOD_NS    (1000000000LL / LIGHT_PWM_FREQ_HZ)   /* 1000000 ns = 1ms */

#define CONTROL_TIMEOUT_MS     1000    /* Reset to defaults after 1s no data */
#define TELEMETRY_INTERVAL_MS  1000    /* Send telemetry every 1s */
#define GPS_INTERVAL_MS        2000    /* Send GPS every 2s */

/* PWM芯片编号 */
#define PWM_CHIP_CH1           10      /* PWM10 - CH1 (电机/电调) */
#define PWM_CHIP_CH2           9       /* PWM9  - CH2 (转向) */
#define PWM_CHIP_CH3           8       /* PWM8  - CH3 (灯光) */
#define PWM_CHIP_CH4           0       /* PWM0  - CH4 (备用) */

/* 模式选择: 0=内置控制, 1=外部电调/控制 */
#define MODE_MOTOR_INTERNAL    0       /* 内置电调: PWM10(FWD)+PWM11(REV) */
#define MODE_MOTOR_EXTERNAL    1       /* 外部电调: PWM10输出CH1舵机信号 */
#define MODE_LIGHT_INTERNAL    0       /* 内置控制: PWM8输出占空比控制MOS管 */
#define MODE_LIGHT_EXTERNAL    1       /* 外部控制: PWM8输出CH3舵机信号 */

/* 默认模式配置 (运行时可通过 TCP 消息切换) */
#define DEFAULT_MOTOR_MODE     MODE_MOTOR_INTERNAL
#define DEFAULT_LIGHT_MODE     MODE_LIGHT_INTERNAL

#define DEFAULT_THROTTLE       512     /* Stop (512对应1.5ms中间值) */
#define DEFAULT_STEERING       512     /* Center */
#define DEFAULT_LIGHT          0       /* Off (0对应0.5ms) */

#define ADC_VOLTAGE_RAW        "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define ADC_VOLTAGE_SCALE      "/sys/bus/iio/devices/iio:device0/in_voltage_scale"

/* ======================== Protocol Definitions ======================== */

#define MAGIC_0    0x5A
#define MAGIC_1    0xA5
#define TYPE_CTRL  0x01
#define TYPE_TELE  0x02
#define TYPE_GPS   0x03
#define TYPE_CONFIG 0x04   /* 配置消息: 设置电调/灯光模式 */

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x01 */
    uint8_t  length;       /* 14 */
    uint16_t throttle;     /* 0~1024, 512=stop */
    uint16_t steering;     /* 0~1024, 512=center */
    uint8_t  light;        /* 0~3 */
    uint32_t tm;           /* Timestamp for latency calculation (ms) */
    uint8_t  checksum;     /* XOR of preceding bytes */
} ControlPacket;           /* 14 bytes */

typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x02 */
    uint8_t  length;       /* 15 */
    int16_t  signal;       /* dBm, 0=not implemented */
    uint32_t voltage_mv;   /* Battery voltage in mV */
    uint32_t tm;           /* Timestamp echoed from ControlPacket */
    uint8_t  checksum;     /* XOR of preceding bytes */
} TelemetryPacket;         /* 15 bytes */

typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x03 */
    uint8_t  length;       /* 30 */
    char     latitude[12]; /* ASCII string, e.g. "39.904200" */
    char     longitude[12];/* ASCII string, e.g. "116.407400" */
    uint8_t  speed;        /* km/h (0~255) */
    uint8_t  checksum;     /* XOR of preceding bytes */
} GpsPacket;               /* 30 bytes */

typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x04 */
    uint8_t  length;       /* 5 */
    uint8_t  motor_mode;   /* 0=内置电调, 1=外部电调 */
    uint8_t  light_mode;   /* 0=内置控制, 1=外部控制 */
    uint8_t  checksum;     /* XOR of preceding bytes */
} ConfigPacket;            /* 5 bytes */
#pragma pack(pop)

/* ======================== Global State ======================== */

static int g_server_fd = -1;
static int g_client_fd = -1;

/* UDP server state */
static int g_udp_server_fd = -1;
static struct sockaddr_in g_udp_client_addr;
static socklen_t g_udp_client_addrlen = sizeof(g_udp_client_addr);
static int g_udp_client_connected = 0;

/* duty_cycle file descriptors, kept open for efficiency */
/* PWM 索引: CH1=电机正转, CH2=转向, CH3=灯光, CH4=备用 */
enum { IDX_CH1 = 0, IDX_CH2, IDX_CH3, IDX_CH4, PWM_COUNT };
static int g_pwm_fd[PWM_COUNT] = { -1, -1, -1, -1 };

/* 内置电调模式专用的 PWM11 反转控制 fd */
static int g_pwm11_fd = -1;

/* 运行时模式控制 (可通过 TCP 消息切换) */
static uint8_t g_motor_mode = DEFAULT_MOTOR_MODE;  /* 0=内置电调, 1=外部电调 */
static uint8_t g_light_mode = DEFAULT_LIGHT_MODE;  /* 0=内置控制, 1=外部控制 */

/* 
 * PWM 引脚映射说明:
 * 
 * 内置电调模式:
 *   - CH1 (PWM10): 电机正转
 *   - CH2 (PWM9):  转向舵机  (固定，不受模式影响)
 *   - CH3 (PWM8):  灯光控制  (PWM占空比/MOS管)
 *   - CH4 (PWM0):  备用舵机
 *   - PWM11:       电机反转  (独立控制，不占用CH索引)
 * 
 * 外部电调模式:
 *   - CH1 (PWM10): 电调舵机信号
 *   - CH2 (PWM9):  转向舵机信号 (固定，不受模式影响)
 *   - CH3 (PWM8):  灯光舵机信号
 *   - CH4 (PWM0):  备用舵机信号
 */

static uint16_t g_throttle = DEFAULT_THROTTLE;
static uint16_t g_steering = DEFAULT_STEERING;
static uint8_t  g_light    = DEFAULT_LIGHT;

/* Motor direction: 1=forward, -1=backward, 0=stop (仅内置电调模式使用) */
static int g_motor_dir = 0;
static int64_t g_dir_change_ms = 0;  /* Direction change timestamp */

static int64_t  g_last_recv_ms = 0;
static volatile int g_running  = 1;

/* ======================== Utilities ======================== */

static uint8_t xor_checksum(const uint8_t *data, int len)
{
    uint8_t cs = 0;
    for (int i = 0; i < len; i++)
        cs ^= data[i];
    return cs;
}

static int64_t time_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static int write_sysfs(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        Printf("[SYS] open %s: %s\n", path, strerror(errno));
        return -1;
    }
    ssize_t n = write(fd, value, strlen(value));
    close(fd);
    return (n < 0) ? -1 : 0;
}

/* ======================== PWM Control ======================== */

static int pwm_export(int chip)
{
    char dir[128], path[128];
    snprintf(dir, sizeof(dir), "/sys/class/pwm/pwmchip%d/pwm0", chip);
    if (access(dir, F_OK) == 0)
        return 0;
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/export", chip);
    if (write_sysfs(path, "0") < 0)
        return -1;
    usleep(100000);
    return 0;
}

static int pwm_init_one(int chip, int64_t period_ns)
{
    char path[128], val[32];

    if (pwm_export(chip) < 0)
        return -1;

    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/period", chip);
    snprintf(val, sizeof(val), "%lld", (long long)period_ns);
    if (write_sysfs(path, val) < 0)
        return -1;

    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/polarity", chip);
    if (write_sysfs(path, "normal") < 0)
        return -1;

    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/duty_cycle", chip);
    if (write_sysfs(path, "0") < 0)
        return -1;

    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/enable", chip);
    if (write_sysfs(path, "1") < 0)
        return -1;

    return 0;
}

static int pwm_open_duty(int chip)
{
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/duty_cycle", chip);
    int fd = open(path, O_WRONLY);
    if (fd < 0)
        Printf("[PWM] open %s: %s\n", path, strerror(errno));
    return fd;
}

static void pwm_set_duty(int fd, int64_t duty_ns)
{
    if (fd < 0)
        return;
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "%lld", (long long)duty_ns);
    lseek(fd, 0, SEEK_SET);
    write(fd, buf, len);
}

static int pwm_init_all(void)
{
    /* 
     * PWM 初始化配置:
     * - CH1 (PWM10): 电机正转 (内置: 500Hz PWM, 外部: 50Hz 舵机)
     * - CH2 (PWM9):  转向舵机 (始终: 50Hz 舵机信号)
     * - CH3 (PWM8):  灯光控制 (内置: 1000Hz PWM, 外部: 50Hz 舵机)
     * - CH4 (PWM0):  备用舵机 (始终: 50Hz 舵机信号)
     * - PWM11:       电机反转 (仅内置电调模式, 500Hz PWM)
     */
    
    /* CH1: PWM10 - 根据电调模式选择周期 */
    int64_t ch1_period = (g_motor_mode == MODE_MOTOR_INTERNAL) ? 
                         MOTOR_PWM_PERIOD_NS : SERVO_PWM_PERIOD_NS;
    
    /* CH2: PWM9 - 始终舵机信号 (转向) */
    int64_t ch2_period = SERVO_PWM_PERIOD_NS;
    
    /* CH3: PWM8 - 根据灯光模式选择周期 */
    int64_t ch3_period = (g_light_mode == MODE_LIGHT_INTERNAL) ? 
                         LIGHT_PWM_PERIOD_NS : SERVO_PWM_PERIOD_NS;
    
    /* CH4: PWM0 - 始终舵机信号 (备用) */
    int64_t ch4_period = SERVO_PWM_PERIOD_NS;
    
    struct { int chip; int idx; int64_t period; } cfg[] = {
        { 10, IDX_CH1, ch1_period },  /* PWM10 - CH1 电调/电机正转 */
        { 9,  IDX_CH2, ch2_period },  /* PWM9  - CH2 转向 (固定) */
        { 8,  IDX_CH3, ch3_period },  /* PWM8  - CH3 灯光 */
        { 0,  IDX_CH4, ch4_period },  /* PWM0  - CH4 备用 */
    };
    
    Printf("[PWM] motor mode: %s, light mode: %s\n",
           g_motor_mode == MODE_MOTOR_INTERNAL ? "internal ESC" : "external servo",
           g_light_mode == MODE_LIGHT_INTERNAL ? "internal MOS" : "external servo");

    /* 初始化 CH1-CH4 */
    for (int i = 0; i < PWM_COUNT; i++) {
        if (pwm_init_one(cfg[i].chip, cfg[i].period) < 0) {
            Printf("[PWM] init chip%d failed\n", cfg[i].chip);
            return -1;
        }
        g_pwm_fd[cfg[i].idx] = pwm_open_duty(cfg[i].chip);
        if (g_pwm_fd[cfg[i].idx] < 0)
            return -1;
    }
    
    /* 内置电调模式: 初始化 PWM11 用于电机反转 */
    if (g_motor_mode == MODE_MOTOR_INTERNAL) {
        if (pwm_init_one(11, MOTOR_PWM_PERIOD_NS) < 0) {
            Printf("[PWM] init PWM11 (motor reverse) failed\n");
            return -1;
        }
        g_pwm11_fd = pwm_open_duty(11);
        if (g_pwm11_fd < 0) {
            Printf("[PWM] open PWM11 duty_cycle failed\n");
            return -1;
        }
        Printf("[PWM] PWM11 initialized for motor reverse\n");
    } else {
        /* 外部电调模式: 关闭 PWM11 */
        if (g_pwm11_fd >= 0) {
            close(g_pwm11_fd);
            g_pwm11_fd = -1;
        }
    }
    
    Printf("[PWM] initialized successfully\n");
    return 0;
}

static void pwm_deinit(void)
{
    for (int i = 0; i < PWM_COUNT; i++) {
        if (g_pwm_fd[i] >= 0) {
            close(g_pwm_fd[i]);
            g_pwm_fd[i] = -1;
        }
    }
    /* 关闭 PWM11 (内置电调模式) */
    if (g_pwm11_fd >= 0) {
        close(g_pwm11_fd);
        g_pwm11_fd = -1;
    }
}

/* ======================== Actuator Output ======================== */

/* 将 0~1024 映射到舵机脉宽 (us) */
static int64_t value_to_servo_us(uint16_t value)
{
    /* 0 → 500us (0.5ms), 512 → 1500us (1.5ms), 1024 → 2500us (2.5ms) */
    return SERVO_MIN_US + (int64_t)value * (SERVO_MAX_US - SERVO_MIN_US) / 1024;
}

static void apply_motor(uint16_t throttle)
{
    if (g_motor_mode == MODE_MOTOR_INTERNAL) {
        /* 内置电调模式: PWM10(CH1)正转 + PWM11独立控制反转 */
        int new_dir;
        int duty;
        int64_t now = time_ms();

        /* 死区处理: 503-521 不输出 */
        if (throttle > (512 + MOTOR_DEAD_ZONE)) {
            new_dir = 1;  /* Forward */
        } else if (throttle < (512 - MOTOR_DEAD_ZONE)) {
            new_dir = -1; /* Backward */
        } else {
            new_dir = 0;  /* Stop */
        }

        /* Within 1s of direction change, keep stopping to protect motor */
        if (g_dir_change_ms > 0 && now - g_dir_change_ms < 1000) {
            new_dir = 0;  /* Force stop */
        }
        /* 1s protection period expired */
        else if (g_dir_change_ms > 0) {
            g_dir_change_ms = 0;
        }

        /* Direction change: forward<->backward, record time and force stop */
        if (g_motor_dir != 0 && new_dir != 0 && g_motor_dir != new_dir) {
            g_dir_change_ms = now;
            new_dir = 0;  /* Force stop */
            /* 立即更新方向，以便保护期后能正确切换 */
            g_motor_dir = new_dir;
        } else {
            /* Normal direction update */
            g_motor_dir = new_dir;
        }

        if (new_dir == 1) {
            /* Forward: PWM11 outputs PWM, PWM10(CH1) fixed low */
            duty = throttle - 512;
            /* 非线性曲线: 前段缓慢上升，后端快速上升 (二次方) */
            /* duty ∈ [0,512] → out ∈ [MOTOR_MIN_DUTY, 512] */
            duty = MOTOR_MIN_DUTY + (int64_t)(512 - MOTOR_MIN_DUTY) * duty * duty / (512 * 512);
            pwm_set_duty(g_pwm_fd[IDX_CH1], 0);
            pwm_set_duty(g_pwm11_fd, (int64_t)duty * MOTOR_PWM_PERIOD_NS / 512);
        } else if (new_dir == -1) {
            /* Backward: PWM10(CH1) outputs PWM, PWM11 fixed low */
            duty = 512 - throttle;
            /* 倒车限速: 占空比上限MOTOR_REV_MAX_DUTY (前进上限512) */
            duty = MOTOR_MIN_DUTY + (int64_t)(MOTOR_REV_MAX_DUTY - MOTOR_MIN_DUTY) * duty * duty / (512 * 512);
            pwm_set_duty(g_pwm_fd[IDX_CH1], (int64_t)duty * MOTOR_PWM_PERIOD_NS / 512);
            pwm_set_duty(g_pwm11_fd, 0);
        } else {
            /* Stop: both low */
            pwm_set_duty(g_pwm_fd[IDX_CH1], 0);
            pwm_set_duty(g_pwm11_fd, 0);
        }
    } else {
        /* 外部电调模式: PWM10(CH1) 输出标准舵机信号 */
        /* 512(中间值) → 1.5ms 停车, 0 → 0.5ms 最大后退, 1024 → 2.5ms 最大前进 */
        int64_t pulse_us = value_to_servo_us(throttle);
        pwm_set_duty(g_pwm_fd[IDX_CH1], pulse_us * 1000);  /* us → ns */
    }
}

static void apply_steering(uint16_t steering)
{
    /* PWM9 始终作为 CH2 转向舵机信号 (不受电机模式影响) */
    /* 二次方曲线: 中位附近缓慢变化，两端快速变化 */
    /* 0 → 2500us, 512 → 1500us, 1024 → 500us */
    int offset = steering - 512;
    int sign = (offset >= 0) ? 1 : -1;
    int abs_offset = sign * offset;  /* |offset|, 范围 [0, 512] */
    /* 二次方映射: abs_offset² / 512² 归一化到 [0,1] */
    int64_t delta_us = (int64_t)(SERVO_MAX_US - SERVO_MIN_US) / 2
                       * abs_offset * abs_offset / (512 * 512);
    int64_t pulse_us = 1500 - sign * delta_us;
    pwm_set_duty(g_pwm_fd[IDX_CH2], pulse_us * 1000);  /* us → ns */
}

static void apply_light(uint8_t light)
{
    if (g_light_mode == MODE_LIGHT_INTERNAL) {
        /* 内置控制模式: PWM8 直接输出占空比控制 MOS 管 */
        static const int64_t duty_table[] = { 0, 200000, 500000, 1000000 };  /* 0%, 20%, 50%, 100% */
        pwm_set_duty(g_pwm_fd[IDX_CH3], duty_table[light > 3 ? 3 : light]);
    } else {
        /* 外部控制模式: PWM8 作为 CH3 标准舵机信号 */
        /* light: 0~3 映射到 0.5ms~2.5ms */
        uint16_t value = (uint32_t)light * 1024 / 3;  /* 0→0, 1→341, 2→682, 3→1024 */
        int64_t pulse_us = value_to_servo_us(value);
        pwm_set_duty(g_pwm_fd[IDX_CH3], pulse_us * 1000);  /* us → ns */
    }
}

static void apply_all(void)
{
    apply_motor(g_throttle);
    apply_steering(g_steering);
    apply_light(g_light);
}

static void set_defaults(void)
{
    g_throttle = DEFAULT_THROTTLE;
    g_steering = DEFAULT_STEERING;
    g_light    = DEFAULT_LIGHT;
    apply_all();
}

/* ======================== ADC / Battery ======================== */

static int read_battery_mv(uint32_t *mv)
{
    FILE *fp;
    int raw;
    double scale;

    fp = fopen(ADC_VOLTAGE_RAW, "r");
    if (!fp) return -1;
    if (fscanf(fp, "%d", &raw) != 1) { fclose(fp); return -1; }
    fclose(fp);

    fp = fopen(ADC_VOLTAGE_SCALE, "r");
    if (!fp) return -1;
    if (fscanf(fp, "%lf", &scale) != 1) { fclose(fp); return -1; }
    fclose(fp);

    *mv = (uint32_t)(raw * scale) * 11;  /* 分压电阻11倍 */
    return 0;
}


/* ======================== Protocol Handling ======================== */

static void build_telemetry(TelemetryPacket *pkt, uint32_t voltage_mv, uint32_t tm)
{
    pkt->magic[0]   = MAGIC_0;
    pkt->magic[1]   = MAGIC_1;
    pkt->type       = TYPE_TELE;
    pkt->length     = sizeof(TelemetryPacket);
    pkt->signal     = ml307c_get_signal();  /* Get signal strength from ML307C */
    pkt->voltage_mv = voltage_mv;
    pkt->tm         = tm;  /* Echo timestamp from ControlPacket */
    pkt->checksum   = xor_checksum((const uint8_t *)pkt, sizeof(TelemetryPacket) - 1);
}

static void build_gps(GpsPacket *pkt, const char *lat, const char *lon, uint8_t spd)
{
    pkt->magic[0]   = MAGIC_0;
    pkt->magic[1]   = MAGIC_1;
    pkt->type       = TYPE_GPS;
    pkt->length     = sizeof(GpsPacket);
    memset(pkt->latitude, 0, sizeof(pkt->latitude));
    memset(pkt->longitude, 0, sizeof(pkt->longitude));
    strncpy(pkt->latitude, lat, sizeof(pkt->latitude) - 1);
    strncpy(pkt->longitude, lon, sizeof(pkt->longitude) - 1);
    pkt->speed      = spd;
    pkt->checksum   = xor_checksum((const uint8_t *)pkt, sizeof(GpsPacket) - 1);
}

/* Parse control packets from buffer. Returns bytes consumed, 0 if no valid frame. */
/* If tm_ptr is provided and tm != 0, store tm value for immediate response */
static int parse_control(const uint8_t *buf, int len, uint32_t *tm_ptr)
{
    for (int i = 0; i <= len - (int)sizeof(ControlPacket); i++) {
        if (buf[i] != MAGIC_0 || buf[i + 1] != MAGIC_1)
            continue;
        if (buf[i + 2] != TYPE_CTRL)
            continue;

        ControlPacket *pkt = (ControlPacket *)(buf + i);
        if (pkt->length != sizeof(ControlPacket))
            continue;
        if (xor_checksum(buf + i, sizeof(ControlPacket) - 1) != pkt->checksum)
            continue;

        /* Valid frame - clamp values to valid ranges */
        g_throttle = pkt->throttle > 1024 ? 1024 : pkt->throttle;
        g_steering = pkt->steering > 1024 ? 1024 : pkt->steering;
        g_light    = pkt->light > 3 ? 3 : pkt->light;
        
        /* Extract tm for immediate response */
        if (tm_ptr && pkt->tm != 0) {
            *tm_ptr = pkt->tm;
        }
        
        {
            static int64_t last_print_ms = 0;
            int64_t now = time_ms();
            if (now - last_print_ms >= 2000) {
                // Printf("[CTRL] throttle=%d, steering=%d, light=%d\n",
                //        g_throttle, g_steering, g_light);
                last_print_ms = now;
            }
        }
        apply_all();
        return i + sizeof(ControlPacket);
    }
    return 0;
}

/* Parse config packets. Returns bytes consumed, 0 if no valid frame. */
static int parse_config(const uint8_t *buf, int len)
{
    for (int i = 0; i <= len - (int)sizeof(ConfigPacket); i++) {
        if (buf[i] != MAGIC_0 || buf[i + 1] != MAGIC_1)
            continue;
        if (buf[i + 2] != TYPE_CONFIG)
            continue;

        ConfigPacket *pkt = (ConfigPacket *)(buf + i);
        if (pkt->length != sizeof(ConfigPacket))
            continue;
        if (xor_checksum(buf + i, sizeof(ConfigPacket) - 1) != pkt->checksum)
            continue;

        /* Valid config frame - update modes */
        if (pkt->motor_mode > 1 || pkt->light_mode > 1) {
            Printf("[CONFIG] invalid mode: motor=%d, light=%d\n", 
                    pkt->motor_mode, pkt->light_mode);
            return i + sizeof(ConfigPacket);
        }

        /* Check if modes changed */
        if (pkt->motor_mode != g_motor_mode || pkt->light_mode != g_light_mode) {
            Printf("[CONFIG] mode change: motor %d->%d, light %d->%d\n",
                   g_motor_mode, pkt->motor_mode,
                   g_light_mode, pkt->light_mode);
            
            /* Update modes */
            g_motor_mode = pkt->motor_mode;
            g_light_mode = pkt->light_mode;
            
            /* Reinitialize PWM with new modes */
            pwm_deinit();
            if (pwm_init_all() < 0) {
                Printf("[CONFIG] PWM reinit failed!\n");
            } else {
                Printf("[CONFIG] PWM reinitialized successfully\n");
            }
            
            /* Reset to defaults after mode change */
            set_defaults();
        }

        return i + sizeof(ConfigPacket);
    }
    return 0;
}

/* ======================== TCP Server ======================== */

static int tcp_init(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { Printf("socket: %s\n", strerror(errno)); return -1; }

    /* Allow socket reuse for quick restart */
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    /* Disable Nagle algorithm for low latency */
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    
    /* Set low latency TOS */
    int tos = 0x10; /* IPTOS_LOWDELAY */
    setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(TCP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        Printf("[TCP] port %d already in use. Kill old process: killall -9 remote_control\n", TCP_PORT);
        Printf("bind: %s\n", strerror(errno)); close(fd); return -1;
    }
    if (listen(fd, 1) < 0) {
        Printf("listen: %s\n", strerror(errno)); close(fd); return -1;
    }

    Printf("[TCP] listening on port %d\n", TCP_PORT);
    return fd;
}

static int udp_init(void)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { Printf("UDP socket: %s\n", strerror(errno)); return -1; }

    /* Allow socket reuse */
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    /* Set low latency TOS */
    int tos = 0x10; /* IPTOS_LOWDELAY */
    setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(TCP_PORT);  /* Same port as TCP */
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        Printf("[UDP] port %d bind failed\n", TCP_PORT);
        Printf("bind: %s\n", strerror(errno)); close(fd); return -1;
    }

    Printf("[UDP] listening on port %d\n", TCP_PORT);
    return fd;
}

static void accept_client(void)
{
    struct sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);
    int fd = accept(g_server_fd, (struct sockaddr *)&addr, &addrlen);
    if (fd < 0) { Printf("accept: %s\n", strerror(errno)); return; }

    /* Disable Nagle algorithm for client socket */
    int opt = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
    
    /* Set low latency TOS for client socket */
    int tos = 0x10; /* IPTOS_LOWDELAY */
    setsockopt(fd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    /* Kick old connection */
    if (g_client_fd >= 0) {
        close(g_client_fd);
        Printf("[TCP] old connection closed\n");
    }
    g_client_fd = fd;
    g_last_recv_ms = time_ms();
    Printf("[TCP] client connected: %s:%d\n",
           inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
}

static void disconnect_client(void)
{
    if (g_client_fd >= 0) {
        close(g_client_fd);
        g_client_fd = -1;
        Printf("[TCP] client disconnected\n");
    }
    set_defaults();
}

static void disconnect_udp_client(void)
{
    if (g_udp_client_connected) {
        g_udp_client_connected = 0;
        Printf("[UDP] client disconnected\n");
    }
    set_defaults();
}

/* ======================== Proxy Config ======================== */

#define PROXY_CONF_PATH "/etc/proxy.conf"

typedef struct {
    char host[64];
    int  port;
} proxy_config_t;

static int parse_proxy_config(proxy_config_t *cfg)
{
    FILE *fp = fopen(PROXY_CONF_PATH, "r");
    if (!fp) {
        Printf("[PROXY] %s not found, rproxyc will not start\n",
                PROXY_CONF_PATH);
        return -1;
    }

    cfg->host[0] = '\0';
    cfg->port = 0;

    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        /* Skip comments and empty lines */
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
            continue;

        /* Parse host=xxx */
        if (strncmp(line, "host=", 5) == 0) {
            char *val = line + 5;
            /* Remove trailing whitespace/newline */
            char *end = val + strlen(val) - 1;
            while (end > val && (*end == '\n' || *end == '\r' || *end == ' '))
                *end-- = '\0';
            if (strlen(val) > 0 && strlen(val) < sizeof(cfg->host))
                strncpy(cfg->host, val, sizeof(cfg->host) - 1);
        }
        /* Parse port=xxx */
        else if (strncmp(line, "port=", 5) == 0) {
            int port = atoi(line + 5);
            if (port > 0 && port < 65536)
                cfg->port = port;
        }
    }

    fclose(fp);

    /* Validate config */
    if (cfg->host[0] == '\0' || cfg->port == 0) {
        Printf("[PROXY] invalid config (host=%s, port=%d), rproxyc will not start\n",
                cfg->host, cfg->port);
        return -1;
    }

    Printf("[PROXY] loaded from %s: %s:%d\n", PROXY_CONF_PATH, cfg->host, cfg->port);
    return 0;
}

/* ======================== Poll Array ======================== */

#define POLL_MAX_FDS    3   /* TCP server, TCP client, UDP server */

enum { IDX_TCP_SERVER = 0, IDX_TCP_CLIENT, IDX_UDP_SERVER };

static struct pollfd g_pollfds[POLL_MAX_FDS];
static int g_poll_nfds = 1;  /* At least TCP server */

/* ======================== Main Loop ======================== */

static void signal_handler(int sig)
{
    g_running = 0;

    Printf("recv signal:%d\n",sig);
    
    /* 强制关闭所有 fd，立即唤醒阻塞的系统调用 */
    if (g_client_fd >= 0) {
        shutdown(g_client_fd, SHUT_RDWR);
        close(g_client_fd);
        g_client_fd = -1;
    }
    if (g_server_fd >= 0) {
        close(g_server_fd);
        g_server_fd = -1;
    }
    if (g_udp_server_fd >= 0) {
        close(g_udp_server_fd);
        g_udp_server_fd = -1;
    }
}

int main(void)
{
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    logInit(0);
    atexit(logDeinit);

    Printf("[MAIN] starting remote_control\n");

    /* Initialize PWM */
    if (pwm_init_all() < 0) {
        Printf("PWM init failed\n");
        return 1;
    }
    set_defaults();

    /* Initialize ML307C (RNDIS + GNSS) */
    if (ml307c_init() < 0)
        Printf("ML307C init failed, GPS will use default\n");

    /* Wait for IMEI and start rproxyc */
    {
        proxy_config_t proxy_cfg;
        if (parse_proxy_config(&proxy_cfg) != 0) {
            Printf("[MAIN] proxy config failed, skip rproxyc\n");
        } else {
            char imei[32] = {0};
            Printf("[MAIN] waiting for IMEI...\n");
            for (int i = 0; i < 120 && g_running; i++) {
                if (ml307c_get_imei(imei, sizeof(imei))) {
                    if (system("pidof rproxyc > /dev/null 2>&1") == 0) {
                        Printf("[MAIN] rproxyc already running, skip\n");
                    } else {
                        Printf("[MAIN] starting rproxyc with SN=%s, proxy=%s:%d\n", 
                               imei, proxy_cfg.host, proxy_cfg.port);
                        char cmd[256];
                        snprintf(cmd, sizeof(cmd),
                                 "/oem/usr/bin/rproxyc %s %s %d &", 
                                 imei, proxy_cfg.host, proxy_cfg.port);
                        system(cmd);
                    }
                    break;
                }
                /* 使用可中断睡眠，每秒检查一次 g_running */
                for (int j = 0; j < 10 && g_running; j++)
                    usleep(100000);  /* 100ms * 10 = 1s */
            }
            if (!g_running) {
                Printf("[MAIN] interrupted during IMEI wait, shutting down\n");
                goto cleanup;
            }
            if (imei[0] == '\0')
                Printf("[MAIN] IMEI timeout, rproxyc not started\n");
        }
    }

    /* Initialize Audio (TCP 5102) */
    if (audio_init() < 0)
        Printf("Audio init failed, intercom will be unavailable\n");
    else
        audio_start();

    /* Initialize TCP server */
    g_server_fd = tcp_init();
    if (g_server_fd < 0) {
        pwm_deinit();
        ml307c_deinit();
        return 1;
    }

    /* Initialize UDP server */
    g_udp_server_fd = udp_init();
    if (g_udp_server_fd < 0) {
        Printf("[UDP] init failed, UDP control disabled\n");
    }


    /* Initialize poll array */
    memset(g_pollfds, 0, sizeof(g_pollfds));
    g_pollfds[IDX_TCP_SERVER].fd = g_server_fd;
    g_pollfds[IDX_TCP_SERVER].events = POLLIN;
    g_pollfds[IDX_TCP_CLIENT].fd = -1;
    g_pollfds[IDX_UDP_SERVER].fd = -1;
    g_poll_nfds = (g_udp_server_fd >= 0) ? IDX_UDP_SERVER + 1 : IDX_TCP_CLIENT + 1;
    if (g_udp_server_fd >= 0) {
        g_pollfds[IDX_UDP_SERVER].fd = g_udp_server_fd;
        g_pollfds[IDX_UDP_SERVER].events = POLLIN;
    }

    uint8_t rx_buf[64];
    int rx_len = 0;
    uint8_t udp_buf[64];  /* UDP receive buffer */
    int64_t last_tele_ms = time_ms();
    int64_t last_gps_ms  = time_ms();

    while (g_running) {
        /* Update poll array fd values (client may connect/disconnect) */
        g_pollfds[IDX_TCP_SERVER].fd = g_server_fd;
        g_pollfds[IDX_TCP_CLIENT].fd = g_client_fd;
        g_pollfds[IDX_UDP_SERVER].fd = g_udp_server_fd;

        int ret = poll(g_pollfds, g_poll_nfds, 10);  /* 10ms timeout */
        if (ret < 0) {
            if (errno == EINTR) continue;
            Printf("poll: %s\n", strerror(errno)); break;
        }

        /* Accept new TCP connection */
        if (g_pollfds[IDX_TCP_SERVER].revents & POLLIN)
            accept_client();

        /* Receive TCP client data */
        if (g_client_fd >= 0 && (g_pollfds[IDX_TCP_CLIENT].revents & POLLIN)) {
            int n = recv(g_client_fd, rx_buf + rx_len, sizeof(rx_buf) - rx_len, 0);
            if (n <= 0) {
                disconnect_client();
                rx_len = 0;
            } else {
                rx_len += n;
                g_last_recv_ms = time_ms();
                
                /* Parse control packets */
                uint32_t response_tm = 0;
                int consumed = parse_control(rx_buf, rx_len, &response_tm);
                if (consumed == 0) {
                    /* Try parse config packets */
                    consumed = parse_config(rx_buf, rx_len);
                }
                
                if (consumed > 0) {
                    rx_len -= consumed;
                    if (rx_len > 0)
                        memmove(rx_buf, rx_buf + consumed, rx_len);
                    
                    /* Immediate response if tm != 0 */
                    if (response_tm != 0) {
                        uint32_t mv = 0;
                        read_battery_mv(&mv);
                        TelemetryPacket pkt;
                        build_telemetry(&pkt, mv, response_tm);
                        if (send(g_client_fd, &pkt, sizeof(pkt), MSG_NOSIGNAL) < 0) {
                            disconnect_client();
                            rx_len = 0;
                        }
                    }
                } else if (rx_len >= (int)sizeof(ControlPacket)) {
                    /* No valid frame found in full buffer, discard */
                    rx_len = 0;
                }
            }
        }

        /* Receive UDP client data */
        if (g_udp_server_fd >= 0 && (g_pollfds[IDX_UDP_SERVER].revents & POLLIN)) {
            int n = recvfrom(g_udp_server_fd, udp_buf, sizeof(udp_buf), 0,
                            (struct sockaddr *)&g_udp_client_addr, &g_udp_client_addrlen);
            if (n > 0) {
                g_udp_client_connected = 1;
                g_last_recv_ms = time_ms();
                
                /* Parse control packets */
                uint32_t response_tm = 0;
                int consumed = parse_control(udp_buf, n, &response_tm);
                
                if (consumed > 0 && response_tm != 0) {
                    /* Immediate response for UDP */
                    uint32_t mv = 0;
                    read_battery_mv(&mv);
                    TelemetryPacket pkt;
                    build_telemetry(&pkt, mv, response_tm);
                    sendto(g_udp_server_fd, &pkt, sizeof(pkt), 0,
                          (struct sockaddr *)&g_udp_client_addr, g_udp_client_addrlen);
                }
            }
        }

        /* Control timeout: 1s no data → reset to defaults */
        /* Check both TCP and UDP clients */
        int any_client_connected = (g_client_fd >= 0) || g_udp_client_connected;
        if (any_client_connected && time_ms() - g_last_recv_ms >= CONTROL_TIMEOUT_MS) {
            set_defaults();
            g_last_recv_ms = time_ms();
            /* Disconnect UDP client on timeout */
            if (g_udp_client_connected) {
                disconnect_udp_client();
            }
        }

        /* Send telemetry every 1s */
        if ((g_client_fd >= 0 || g_udp_client_connected) && time_ms() - last_tele_ms >= TELEMETRY_INTERVAL_MS) {
            uint32_t mv = 0;
            read_battery_mv(&mv);
            TelemetryPacket pkt;
            build_telemetry(&pkt, mv, 0);  /* tm=0 for periodic telemetry */
            
            /* Send via TCP if connected */
            if (g_client_fd >= 0) {
                if (send(g_client_fd, &pkt, sizeof(pkt), MSG_NOSIGNAL) < 0)
                    disconnect_client();
            }
            
            /* Send via UDP if connected */
            if (g_udp_client_connected && g_udp_server_fd >= 0) {
                sendto(g_udp_server_fd, &pkt, sizeof(pkt), 0,
                      (struct sockaddr *)&g_udp_client_addr, g_udp_client_addrlen);
            }
            
            last_tele_ms = time_ms();
        }

        /* Send GPS every 2s */
        if (g_client_fd >= 0 && time_ms() - last_gps_ms >= GPS_INTERVAL_MS) {
            GpsPacket gps;
            ml307c_gps_data_t gpsd;
            if (ml307c_get_gps(&gpsd)) {
                char lat[12], lon[12];
                snprintf(lat, sizeof(lat), "%.6f", gpsd.latitude);
                snprintf(lon, sizeof(lon), "%.6f", gpsd.longitude);
                uint8_t spd = (gpsd.speed_kmh > 255.0f) ? 255 :
                              (uint8_t)(gpsd.speed_kmh + 0.5f);
                build_gps(&gps, lat, lon, spd);
            } else {
                build_gps(&gps, "0.000000", "0.000000", 0);
            }
            if (send(g_client_fd, &gps, sizeof(gps), MSG_NOSIGNAL) < 0)
                disconnect_client();
            last_gps_ms = time_ms();
        }
    }

    /* Cleanup - order matters: stop threads first */
    Printf("\n[MAIN] shutting down...\n");
    disconnect_client();
    
cleanup:
    /* Stop audio threads */
    audio_stop();
    
    /* Close server fd to unblock poll() if waiting */
    if (g_server_fd >= 0) {
        close(g_server_fd);
        g_server_fd = -1;
    }
    
    /* Close UDP server fd */
    if (g_udp_server_fd >= 0) {
        close(g_udp_server_fd);
        g_udp_server_fd = -1;
    }
    
    /* Stop ML307C threads (monitor + reader) */
    ml307c_deinit();
    
    /* Cleanup audio */
    audio_cleanup();
    
    /* Cleanup PWM */
    pwm_deinit();
    
    Printf("[MAIN] Server stopped\n");
    return 0;
}
