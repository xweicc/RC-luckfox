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
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include "ml307c.h"
#include "audio.h"


/* ======================== Configuration ======================== */

#define TCP_PORT                5103
#define MOTOR_PWM_FREQ_HZ      500     /* Motor PWM frequency */
#define STEER_PWM_FREQ_HZ      50      /* Servo PWM frequency */
#define LIGHT_PWM_FREQ_HZ      1000    /* Light PWM frequency */

#define MOTOR_PWM_PERIOD_NS    (1000000000LL / MOTOR_PWM_FREQ_HZ)   /* 2000000  */
#define STEER_PWM_PERIOD_NS    (1000000000LL / STEER_PWM_FREQ_HZ)   /* 20000000 */
#define LIGHT_PWM_PERIOD_NS    (1000000000LL / LIGHT_PWM_FREQ_HZ)   /* 1000000  */

#define CONTROL_TIMEOUT_MS     1000    /* Reset to defaults after 1s no data */
#define TELEMETRY_INTERVAL_MS  1000    /* Send telemetry every 1s */
#define GPS_INTERVAL_MS        2000    /* Send GPS every 2s */

#define PWM_CHIP_MOTOR_FWD     8       /* PWM8  - motor forward  */
#define PWM_CHIP_MOTOR_REV     9       /* PWM9  - motor reverse  */
#define PWM_CHIP_STEER         10      /* PWM10 - steering servo */
#define PWM_CHIP_LIGHT         0      /* PWM11 - light          */

#define DEFAULT_THROTTLE       512     /* Stop */
#define DEFAULT_STEERING       512     /* Center */
#define DEFAULT_LIGHT          0       /* Off */

#define ADC_VOLTAGE_RAW        "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define ADC_VOLTAGE_SCALE      "/sys/bus/iio/devices/iio:device0/in_voltage_scale"

/* ======================== Protocol Definitions ======================== */

#define MAGIC_0    0x5A
#define MAGIC_1    0xA5
#define TYPE_CTRL  0x01
#define TYPE_TELE  0x02
#define TYPE_GPS   0x03

#pragma pack(push, 1)
typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x01 */
    uint8_t  length;       /* 10 */
    uint16_t throttle;     /* 0~1024, 512=stop */
    uint16_t steering;     /* 0~1024, 512=center */
    uint8_t  light;        /* 0~3 */
    uint8_t  checksum;     /* XOR of preceding bytes */
} ControlPacket;           /* 10 bytes */

typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x02 */
    uint8_t  length;       /* 11 */
    int16_t  signal;       /* dBm, 0=not implemented */
    uint32_t voltage_mv;   /* Battery voltage in mV */
    uint8_t  checksum;     /* XOR of preceding bytes */
} TelemetryPacket;         /* 11 bytes */

typedef struct {
    uint8_t  magic[2];     /* 0x5A, 0xA5 */
    uint8_t  type;         /* 0x03 */
    uint8_t  length;       /* 30 */
    char     latitude[12]; /* ASCII string, e.g. "39.904200" */
    char     longitude[12];/* ASCII string, e.g. "116.407400" */
    uint8_t  speed;        /* km/h (0~255) */
    uint8_t  checksum;     /* XOR of preceding bytes */
} GpsPacket;               /* 30 bytes */
#pragma pack(pop)

/* ======================== Global State ======================== */

static int g_server_fd = -1;
static int g_client_fd = -1;

/* duty_cycle file descriptors, kept open for efficiency */
enum { IDX_FWD = 0, IDX_REV, IDX_STEER, IDX_LIGHT, PWM_COUNT };
static int g_pwm_fd[PWM_COUNT] = { -1, -1, -1, -1 };

static uint16_t g_throttle = DEFAULT_THROTTLE;
static uint16_t g_steering = DEFAULT_STEERING;
static uint8_t  g_light    = DEFAULT_LIGHT;

/* Motor direction: 1=forward, -1=backward, 0=stop */
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
        fprintf(stderr, "[SYS] open %s: %s\n", path, strerror(errno));
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
        fprintf(stderr, "[PWM] open %s: %s\n", path, strerror(errno));
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
    struct { int chip; int idx; int64_t period; } cfg[] = {
        { PWM_CHIP_MOTOR_FWD, IDX_FWD,   MOTOR_PWM_PERIOD_NS },
        { PWM_CHIP_MOTOR_REV, IDX_REV,   MOTOR_PWM_PERIOD_NS },
        { PWM_CHIP_STEER,     IDX_STEER, STEER_PWM_PERIOD_NS },
        { PWM_CHIP_LIGHT,     IDX_LIGHT, LIGHT_PWM_PERIOD_NS },
    };

    for (int i = 0; i < PWM_COUNT; i++) {
        if (pwm_init_one(cfg[i].chip, cfg[i].period) < 0) {
            fprintf(stderr, "[PWM] init chip%d failed\n", cfg[i].chip);
            return -1;
        }
        g_pwm_fd[cfg[i].idx] = pwm_open_duty(cfg[i].chip);
        if (g_pwm_fd[cfg[i].idx] < 0)
            return -1;
    }
    printf("[PWM] initialized (motor=%dHz, steer=%dHz, light=%dHz)\n",
           MOTOR_PWM_FREQ_HZ, STEER_PWM_FREQ_HZ, LIGHT_PWM_FREQ_HZ);
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
}

/* ======================== Actuator Output ======================== */

static void apply_motor(uint16_t throttle)
{
    int new_dir;
    int64_t now = time_ms();

    if (throttle > 512)
        new_dir = 1;       /* Forward */
    else if (throttle < 512)
        new_dir = -1;      /* Backward */
    else
        new_dir = 0;       /* Stop */

    /* Within 1s of direction change, keep stopping to protect motor */
    if (g_dir_change_ms > 0 && now - g_dir_change_ms < 1000) {
        new_dir = 0;  /* Force stop */
    }
    /* 1s protection period expired, update direction */
    else if (g_dir_change_ms > 0) {
        g_motor_dir = new_dir;
        g_dir_change_ms = 0;
    }

    /* Direction change: forward<->backward, record time and force stop */
    if (g_motor_dir != 0 && new_dir != 0 && g_motor_dir != new_dir) {
        g_dir_change_ms = now;
        new_dir = 0;  /* Force stop */
    }

    g_motor_dir = new_dir;

    if (new_dir == 1) {
        /* Forward: PWM8 outputs PWM, PWM9 fixed low */
        pwm_set_duty(g_pwm_fd[IDX_FWD], (int64_t)(throttle - 512) * MOTOR_PWM_PERIOD_NS / 512);
        pwm_set_duty(g_pwm_fd[IDX_REV], 0);
    } else if (new_dir == -1) {
        /* Backward: PWM9 outputs PWM, PWM8 fixed low */
        pwm_set_duty(g_pwm_fd[IDX_FWD], 0);
        pwm_set_duty(g_pwm_fd[IDX_REV], (int64_t)(512 - throttle) * MOTOR_PWM_PERIOD_NS / 512);
    } else {
        /* Stop: both low */
        pwm_set_duty(g_pwm_fd[IDX_FWD], 0);
        pwm_set_duty(g_pwm_fd[IDX_REV], 0);
    }
}

static void apply_steering(uint16_t steering)
{
    /* Pulse width: 0.5ms(0) ~ 1.5ms(512) ~ 2.5ms(1024) */
    int64_t duty = 500000LL + (int64_t)steering * 2000000LL / 1024;
    pwm_set_duty(g_pwm_fd[IDX_STEER], duty);
}

static void apply_light(uint8_t light)
{
    static const int64_t duty_table[] = { 0, 200000, 500000, 1000000 };
    pwm_set_duty(g_pwm_fd[IDX_LIGHT], duty_table[light > 3 ? 3 : light]);
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

static void build_telemetry(TelemetryPacket *pkt, uint32_t voltage_mv)
{
    pkt->magic[0]   = MAGIC_0;
    pkt->magic[1]   = MAGIC_1;
    pkt->type       = TYPE_TELE;
    pkt->length     = sizeof(TelemetryPacket);
    pkt->signal     = ml307c_get_signal();  /* Get signal strength from ML307C */
    pkt->voltage_mv = voltage_mv;
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
static int parse_control(const uint8_t *buf, int len)
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
        {
            static int64_t last_print_ms = 0;
            int64_t now = time_ms();
            if (now - last_print_ms >= 2000) {
                // printf("[CTRL] throttle=%d, steering=%d, light=%d\n",
                //        g_throttle, g_steering, g_light);
                last_print_ms = now;
            }
        }
        apply_all();
        return i + sizeof(ControlPacket);
    }
    return 0;
}

/* ======================== TCP Server ======================== */

static int tcp_init(void)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }

    /* Allow socket reuse for quick restart */
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(TCP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[TCP] port %d already in use. Kill old process: killall -9 remote_control\n", TCP_PORT);
        perror("bind"); close(fd); return -1;
    }
    if (listen(fd, 1) < 0) {
        perror("listen"); close(fd); return -1;
    }

    printf("[TCP] listening on port %d\n", TCP_PORT);
    return fd;
}

static void accept_client(void)
{
    struct sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);
    int fd = accept(g_server_fd, (struct sockaddr *)&addr, &addrlen);
    if (fd < 0) { perror("accept"); return; }

    /* Kick old connection */
    if (g_client_fd >= 0) {
        close(g_client_fd);
        printf("[TCP] old connection closed\n");
    }
    g_client_fd = fd;
    g_last_recv_ms = time_ms();
    printf("[TCP] client connected: %s:%d\n",
           inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
}

static void disconnect_client(void)
{
    if (g_client_fd >= 0) {
        close(g_client_fd);
        g_client_fd = -1;
        printf("[TCP] client disconnected\n");
    }
    set_defaults();
}

/* ======================== Main Loop ======================== */

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

int main(void)
{
    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    /* Initialize PWM */
    if (pwm_init_all() < 0) {
        fprintf(stderr, "PWM init failed\n");
        return 1;
    }
    set_defaults();

    /* Initialize ML307C (RNDIS + GNSS) */
    if (ml307c_init() < 0)
        fprintf(stderr, "ML307C init failed, GPS will use default\n");

    /* Initialize Audio (TCP 5102) */
    if (audio_init() < 0)
        fprintf(stderr, "Audio init failed, intercom will be unavailable\n");
    else
        audio_start();

    /* Initialize TCP server */
    g_server_fd = tcp_init();
    if (g_server_fd < 0) {
        pwm_deinit();
        ml307c_deinit();
        return 1;
    }


    uint8_t rx_buf[64];
    int rx_len = 0;
    int64_t last_tele_ms = time_ms();
    int64_t last_gps_ms  = time_ms();

    while (g_running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(g_server_fd, &fds);
        int maxfd = g_server_fd;
        if (g_client_fd >= 0) {
            FD_SET(g_client_fd, &fds);
            if (g_client_fd > maxfd) maxfd = g_client_fd;
        }

        struct timeval tv = { 0, 50000 }; /* 50ms */
        int ret = select(maxfd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select"); break;
        }

        /* Accept new connection */
        if (FD_ISSET(g_server_fd, &fds))
            accept_client();

        /* Receive client data */
        if (g_client_fd >= 0 && FD_ISSET(g_client_fd, &fds)) {
            int n = recv(g_client_fd, rx_buf + rx_len, sizeof(rx_buf) - rx_len, 0);
            if (n <= 0) {
                disconnect_client();
                rx_len = 0;
            } else {
                rx_len += n;
                g_last_recv_ms = time_ms();
                int consumed = parse_control(rx_buf, rx_len);
                if (consumed > 0) {
                    rx_len -= consumed;
                    if (rx_len > 0)
                        memmove(rx_buf, rx_buf + consumed, rx_len);
                } else if (rx_len >= (int)sizeof(ControlPacket)) {
                    /* No valid frame found in full buffer, discard */
                    rx_len = 0;
                }
            }
        }

        /* Control timeout: 1s no data → reset to defaults */
        if (g_client_fd >= 0 && time_ms() - g_last_recv_ms >= CONTROL_TIMEOUT_MS) {
            set_defaults();
            g_last_recv_ms = time_ms();
        }

        /* Send telemetry every 1s */
        if (g_client_fd >= 0 && time_ms() - last_tele_ms >= TELEMETRY_INTERVAL_MS) {
            uint32_t mv = 0;
            read_battery_mv(&mv);
            TelemetryPacket pkt;
            build_telemetry(&pkt, mv);
            if (send(g_client_fd, &pkt, sizeof(pkt), MSG_NOSIGNAL) < 0)
                disconnect_client();
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
    printf("\n[MAIN] shutting down...\n");
    disconnect_client();
    
    /* Stop audio threads */
    audio_stop();
    
    /* Close server fd to unblock select() if waiting */
    if (g_server_fd >= 0) {
        close(g_server_fd);
        g_server_fd = -1;
    }
    
    /* Stop ML307C threads (monitor + reader) */
    ml307c_deinit();
    
    /* Cleanup audio */
    audio_cleanup();
    
    /* Cleanup PWM */
    pwm_deinit();
    
    printf("[MAIN] Server stopped\n");
    return 0;
}
