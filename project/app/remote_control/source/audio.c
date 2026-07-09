/*
 * audio.c - 双向对讲音频模块（ALSA 版本）
 *
 * 功能：
 * - 监听 TCP 5102 端口
 * - 接收客户端连接后，发送麦克风采集的音频流
 * - 接收客户端的音频流，解码后播放
 * - 播放期间暂停录音，避免回声（无 AEC）
 * - 音频收发在独立线程处理
 *
 * 音频格式：
 *   硬件侧：16kHz / 立体声(硬件要求) / 16bit PCM
 *   网络侧：USE_G711A=1 时 16kHz/mono/G.711a (320 bytes/frame)
 *            USE_G711A=0 时 16kHz/mono/PCM16LE (640 bytes/frame)
 * 底层接口：ALSA (libasound)，不依赖 Rockchip MPI
 *
 * 使用方式：
 *   audio_init()    - 初始化音频和 TCP 服务器
 *   audio_start()   - 启动音频服务线程
 *   audio_stop()    - 停止音频服务
 *   audio_cleanup() - 清理资源
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <alsa/asoundlib.h>
#include "log.h"

/* ======================== 配置参数 ======================== */

#define TCP_AUDIO_PORT          5102
#define AUDIO_SAMPLE_RATE       16000   /* 16kHz */
#define AUDIO_CHANNELS          2       /* 立体声 (RV1103 声卡硬件要求必须为 2 声道) */
#define AUDIO_BIT_WIDTH         16      /* 16bit */
#define AUDIO_FRAME_SAMPLES     320     /* 每帧采样点数 (20ms @ 16kHz) */
#define AUDIO_BUFFER_BYTES      (AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS * (AUDIO_BIT_WIDTH / 8))

/* 客户端音频格式 */
#define CLIENT_AUDIO_CHANNELS   1       /* 客户端发送单声道 */
#define USE_G711A               1       /* 1=G.711a编码(320B/帧), 0=原始PCM16LE(640B/帧) */

#if USE_G711A
#define CLIENT_BYTES_PER_SAMPLE 1       /* G.711a: 每个采样点 1 byte */
#else
#define CLIENT_BYTES_PER_SAMPLE 2       /* 原始 PCM: 每个采样点 2 bytes (16bit) */
#endif
#define CLIENT_BUFFER_BYTES     (AUDIO_FRAME_SAMPLES * CLIENT_AUDIO_CHANNELS * CLIENT_BYTES_PER_SAMPLE)

/* ALSA 设备名称 */
#define ALSA_PCM_DEVICE         "hw:0,0"

/* 功放控制 GPIO */
#define SPK_CTRL_GPIO           59      /* 功放控制引脚，高电平有效 */
#define __SPK_CTRL_GPIO_STR     "59"    /* 字符串形式 */

/* ======================== 日志宏 ======================== */

#define LOG_INFO(fmt, ...)  Printf("[Audio][I] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  Printf("[Audio][W] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)   Printf("[Audio][E] " fmt "\n", ##__VA_ARGS__)

/* ======================== G.711a (A-law) 编解码 ======================== */

#if USE_G711A

/* G.711a 编码：16-bit 线性 PCM → 8-bit A-law (ITU-T G.711) */
static uint8_t pcm_to_alaw(int16_t pcm_val) {
    int pcm = pcm_val;
    uint8_t sign;
    int seg;

    if (pcm < 0) {
        sign = 0x80;
        pcm = -pcm;
    } else {
        sign = 0x00;
    }
    if (pcm > 32767) pcm = 32767;

    /* 确定段号 (ITU-T 标准: seg 0-7) */
    if (pcm < 256) {
        seg = 0;
    } else {
        seg = 1;
        while (seg < 7 && pcm >= (256 << seg)) seg++;
    }

    /* 提取段内 4-bit 量化值 */
    int mantissa;
    if (seg == 0) {
        mantissa = (pcm >> 4) & 0x0F;
    } else {
        mantissa = (pcm >> (seg + 1)) & 0x0F;
    }

    int aval = (seg << 4) | mantissa;
    return (uint8_t)(sign | (aval ^ 0x55));
}

/* G.711a 解码：8-bit A-law → 16-bit 线性 PCM (ITU-T G.711) */
static int16_t alaw_to_pcm(uint8_t alaw_val) {
    int sign, seg, mantissa;
    int pcm;

    /* 先提取符号位，再 XOR 还原数据位 */
    sign = (alaw_val & 0x80) ? 0x80 : 0x00;
    alaw_val ^= 0x55;
    int aval = alaw_val & 0x7F;

    seg = (aval >> 4) & 0x07;
    mantissa = aval & 0x0F;

    if (seg == 0) {
        pcm = (mantissa << 4) + 8;
    } else {
        pcm = ((mantissa << 4) + 0x108) << (seg - 1);
    }

    return (int16_t)(sign ? -pcm : pcm);
}

#endif /* USE_G711A */

/* ======================== GPIO 控制 ======================== */

static int gpio_write_sysfs(const char *path, const char *value) {
    FILE *fp = fopen(path, "w");
    if (!fp) {
        LOG_ERR("GPIO: open %s failed: %s", path, strerror(errno));
        return -1;
    }
    fprintf(fp, "%s", value);
    fclose(fp);
    return 0;
}

static int spk_ctrl_init(void) {
    char path[64];

    snprintf(path, sizeof(path), "/sys/class/gpio/export");
    if (gpio_write_sysfs(path, __SPK_CTRL_GPIO_STR) != 0) {
        LOG_WARN("GPIO: export gpio%d (may already exported)", SPK_CTRL_GPIO);
    }
    usleep(100000);

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", SPK_CTRL_GPIO);
    if (gpio_write_sysfs(path, "out") != 0) {
        LOG_ERR("GPIO: set gpio%d direction failed", SPK_CTRL_GPIO);
        return -1;
    }

    LOG_INFO("GPIO: speaker control gpio%d initialized", SPK_CTRL_GPIO);
    return 0;
}

static void spk_ctrl_enable(void) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", SPK_CTRL_GPIO);
    if (gpio_write_sysfs(path, "1") == 0) {
        LOG_INFO("GPIO: speaker amplifier enabled (gpio%d=HIGH)", SPK_CTRL_GPIO);
    }
}

static void spk_ctrl_disable(void) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", SPK_CTRL_GPIO);
    if (gpio_write_sysfs(path, "0") == 0) {
        LOG_INFO("GPIO: speaker amplifier disabled (gpio%d=LOW)", SPK_CTRL_GPIO);
    }
}

/* ======================== 全局状态 ======================== */

static volatile int g_audio_running = 0;
static volatile int g_is_playing = 0;   /* 播放中标识，send 线程据此暂停录音 */
static volatile int g_capture_enabled = 0;  /* Capture enabled flag for lazy startup */
static volatile int g_playback_enabled = 0; /* Playback enabled flag for lazy startup */
static int g_client_fd = -1;
static int g_server_fd = -1;
static pthread_t g_audio_thread_id = 0;

static snd_pcm_t *g_pcm_capture = NULL;
static snd_pcm_t *g_pcm_playback = NULL;

/* ======================== ALSA 设备配置 ======================== */

/**
 * 配置 ALSA PCM 设备参数
 *
 * @param pcm       ALSA PCM 句柄
 * @param stream    流方向（SND_PCM_STREAM_CAPTURE / SND_PCM_STREAM_PLAYBACK）
 * @return 0 成功，-1 失败
 */
static int alsa_set_params(snd_pcm_t *pcm, snd_pcm_stream_t stream) {
    snd_pcm_hw_params_t *hw_params;
    int ret;

    snd_pcm_hw_params_alloca(&hw_params);

    ret = snd_pcm_hw_params_any(pcm, hw_params);
    if (ret < 0) {
        LOG_ERR("hw_params_any failed: %s", snd_strerror(ret));
        return -1;
    }

    ret = snd_pcm_hw_params_set_access(pcm, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (ret < 0) {
        LOG_ERR("set_access failed: %s", snd_strerror(ret));
        return -1;
    }

    ret = snd_pcm_hw_params_set_format(pcm, hw_params, SND_PCM_FORMAT_S16_LE);
    if (ret < 0) {
        LOG_ERR("set_format failed: %s", snd_strerror(ret));
        return -1;
    }

    unsigned int rate = AUDIO_SAMPLE_RATE;
    ret = snd_pcm_hw_params_set_rate_near(pcm, hw_params, &rate, NULL);
    if (ret < 0) {
        LOG_ERR("set_rate failed: %s", snd_strerror(ret));
        return -1;
    }
    if (rate != AUDIO_SAMPLE_RATE) {
        LOG_WARN("rate %dHz not available, using %dHz", AUDIO_SAMPLE_RATE, rate);
    }

    ret = snd_pcm_hw_params_set_channels(pcm, hw_params, AUDIO_CHANNELS);
    if (ret < 0) {
        LOG_ERR("set_channels(%d) failed: %s", AUDIO_CHANNELS, snd_strerror(ret));
        return -1;
    }

    /* 设置周期大小 = 1 帧 (320 samples = 20ms) */
    snd_pcm_uframes_t period_size = AUDIO_FRAME_SAMPLES;
    ret = snd_pcm_hw_params_set_period_size_near(pcm, hw_params, &period_size, NULL);
    if (ret < 0) {
        LOG_ERR("set_period_size failed: %s", snd_strerror(ret));
        return -1;
    }

    /* 设置缓冲大小 = 16 个周期 (320ms)，增强抗网络抖动能力 */
    snd_pcm_uframes_t buffer_size = period_size * 16;
    ret = snd_pcm_hw_params_set_buffer_size_near(pcm, hw_params, &buffer_size);
    if (ret < 0) {
        LOG_ERR("set_buffer_size failed: %s", snd_strerror(ret));
        return -1;
    }

    ret = snd_pcm_hw_params(pcm, hw_params);
    if (ret < 0) {
        LOG_ERR("hw_params failed: %s", snd_strerror(ret));
        return -1;
    }

    LOG_INFO("ALSA %s: rate=%dHz, channels=%d, period=%lu, buffer=%lu (%.1fms)",
             stream == SND_PCM_STREAM_CAPTURE ? "capture" : "playback",
             rate, AUDIO_CHANNELS,
             (unsigned long)period_size, (unsigned long)buffer_size,
             (double)buffer_size / rate * 1000);
    return 0;
}

/* ======================== ALSA Mixer 控制 ======================== */

/**
 * 设置 ALSA mixer 控件（通过 amixer 命令行）
 */
static void alsa_mixer_set(const char *control, const char *value) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "amixer set '%s' %s >/dev/null 2>&1", control, value);
    int ret = system(cmd);
    if (ret != 0) {
        LOG_WARN("amixer set '%s' %s failed (ret=%d)", control, value, ret);
    } else {
        LOG_INFO("amixer set '%s' %s OK", control, value);
    }
}

/**
 * 配置采集端增益参数
 */
static void alsa_capture_setup_mixer(void) {
    alsa_mixer_set("ADC ALC Left", "26");
    alsa_mixer_set("ADC ALC Right", "26");
    alsa_mixer_set("ADC MIC Left Gain", "3");
    alsa_mixer_set("ADC MIC Right Gain", "3");
    alsa_mixer_set("ADC MICBIAS Voltage", "VREFx0_975");
    alsa_mixer_set("ADC Mode", "SingadcL");
}

/* ======================== 音频输入初始化 ======================== */

static int audio_capture_init(void) {
    /* 仅初始化功放 GPIO（但不开启） */
    spk_ctrl_init();
    LOG_INFO("ALSA capture GPIO initialized (lazy open on client connect)");
    return 0;
}

/* ======================== 音频输出初始化 ======================== */

static int audio_playback_init(void) {
    /* 仅初始化占位，实际打开延迟到客户端连接 */
    LOG_INFO("ALSA playback initialized (lazy open on client connect)");
    return 0;
}

/**
 * Enable audio capture (called on client connect) - lazy startup to save CPU
 */
static int audio_capture_enable(void) {
    int ret;

    if (g_capture_enabled) {
        LOG_WARN("Capture already enabled");
        return 0;
    }

    ret = snd_pcm_open(&g_pcm_capture, ALSA_PCM_DEVICE,
                       SND_PCM_STREAM_CAPTURE, 0);
    if (ret < 0) {
        LOG_ERR("capture open '%s' failed: %s", ALSA_PCM_DEVICE, snd_strerror(ret));
        return -1;
    }

    if (alsa_set_params(g_pcm_capture, SND_PCM_STREAM_CAPTURE) != 0) {
        snd_pcm_close(g_pcm_capture);
        g_pcm_capture = NULL;
        return -1;
    }

    ret = snd_pcm_prepare(g_pcm_capture);
    if (ret < 0) {
        LOG_ERR("capture prepare failed: %s", snd_strerror(ret));
        snd_pcm_close(g_pcm_capture);
        g_pcm_capture = NULL;
        return -1;
    }

    /* 配置采集端增益（ALC + MIC Gain） */
    alsa_capture_setup_mixer();

    g_capture_enabled = 1;
    LOG_INFO("=== ALSA capture enabled (lazy startup on client connect) ===");
    return 0;
}

/**
 * Disable audio capture (called on client disconnect) - save CPU when idle
 */
static void audio_capture_disable(void) {
    if (!g_capture_enabled) {
        return;
    }

    LOG_INFO("Disabling ALSA capture (client disconnected)...");
    if (g_pcm_capture) {
        snd_pcm_drop(g_pcm_capture);
        snd_pcm_close(g_pcm_capture);
        g_pcm_capture = NULL;
    }
    g_capture_enabled = 0;
    LOG_INFO("=== ALSA capture disabled (save CPU) ===");
}

/**
 * Enable audio playback (called on client connect) - lazy startup
 */
static int audio_playback_enable(void) {
    int ret;

    if (g_playback_enabled) {
        LOG_WARN("Playback already enabled");
        return 0;
    }

    ret = snd_pcm_open(&g_pcm_playback, ALSA_PCM_DEVICE,
                       SND_PCM_STREAM_PLAYBACK, 0);
    if (ret < 0) {
        LOG_ERR("playback open '%s' failed: %s", ALSA_PCM_DEVICE, snd_strerror(ret));
        return -1;
    }

    if (alsa_set_params(g_pcm_playback, SND_PCM_STREAM_PLAYBACK) != 0) {
        snd_pcm_close(g_pcm_playback);
        g_pcm_playback = NULL;
        return -1;
    }

    ret = snd_pcm_prepare(g_pcm_playback);
    if (ret < 0) {
        LOG_ERR("playback prepare failed: %s", snd_strerror(ret));
        snd_pcm_close(g_pcm_playback);
        g_pcm_playback = NULL;
        return -1;
    }

    g_playback_enabled = 1;
    LOG_INFO("=== ALSA playback enabled (lazy startup on client connect) ===");
    return 0;
}

/**
 * Disable audio playback (called on client disconnect) - save resources when idle
 */
static void audio_playback_disable(void) {
    if (!g_playback_enabled) {
        return;
    }

    LOG_INFO("Disabling ALSA playback (client disconnected)...");
    if (g_pcm_playback) {
        snd_pcm_drop(g_pcm_playback);
        snd_pcm_close(g_pcm_playback);
        g_pcm_playback = NULL;
    }
    g_playback_enabled = 0;
    LOG_INFO("=== ALSA playback disabled (save resources) ===");
}

/* ======================== 发送音频线程（采集 → 客户端） ======================== */

static void *audio_send_thread(void *arg) {
    char stereo_buf[AUDIO_BUFFER_BYTES];  /* 立体声采集缓冲区 (1280 bytes) */
    int16_t mono_samples[AUDIO_FRAME_SAMPLES];  /* 单声道 PCM (640 bytes) */
#if USE_G711A
    uint8_t client_buf[CLIENT_BUFFER_BYTES];    /* G.711a 编码缓冲区 (320 bytes) */
#else
    uint8_t *client_buf = (uint8_t *)mono_samples;  /* 原始 PCM: 直接发送单声道 (640 bytes) */
#endif
    snd_pcm_sframes_t frames;

    LOG_INFO("Audio send thread started");
    LOG_INFO("Capture format: 16kHz/%dch/%dbit (%zu bytes/frame)",
             AUDIO_CHANNELS, AUDIO_BIT_WIDTH, (size_t)AUDIO_BUFFER_BYTES);
#if USE_G711A
    LOG_INFO("Send format: 16kHz/mono/G.711a (%zu bytes/frame)",
             (size_t)CLIENT_BUFFER_BYTES);
#else
    LOG_INFO("Send format: 16kHz/mono/PCM16LE (%zu bytes/frame)",
             (size_t)CLIENT_BUFFER_BYTES);
#endif

    while (g_audio_running && g_client_fd >= 0) {
        /* 从 ALSA 采集立体声数据（阻塞） */
        frames = snd_pcm_readi(g_pcm_capture, stereo_buf, AUDIO_FRAME_SAMPLES);
        if (frames < 0) {
            if (frames == -EPIPE) {
                LOG_WARN("Capture overrun, recovering...");
                snd_pcm_prepare(g_pcm_capture);
                continue;
            }
            LOG_ERR("Capture read error: %s", snd_strerror(frames));
            break;
        }

        /* 播放中暂停发送，避免回声（无 AEC） */
        if (g_is_playing) {
            continue;
        }

        /* 立体声 → 单声道（只使用左声道，RV1103 MIC0P 是左声道） */
        int16_t *stereo_samples = (int16_t *)stereo_buf;
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            mono_samples[i] = stereo_samples[i * 2];
        }

        /* 单声道 PCM → 客户端格式 */
#if USE_G711A
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            client_buf[i] = pcm_to_alaw(mono_samples[i]);
        }
#endif

        /* 发送数据到客户端 */
        ssize_t sent = send(g_client_fd, client_buf, CLIENT_BUFFER_BYTES, MSG_NOSIGNAL);
        if (sent < 0) {
            LOG_ERR("Send audio failed: %s", strerror(errno));
            break;
        } else if ((size_t)sent != CLIENT_BUFFER_BYTES) {
            LOG_WARN("Partial send: %zd/%zu bytes", sent, (size_t)CLIENT_BUFFER_BYTES);
        }
    }

    LOG_INFO("Audio send thread exited");
    return NULL;
}

/* ======================== 接收音频线程（客户端 → 播放） ======================== */

static void *audio_recv_thread(void *arg) {
    char stereo_buf[AUDIO_BUFFER_BYTES];  /* 立体声缓冲区 (1280 bytes) */
    uint8_t client_buf[CLIENT_BUFFER_BYTES];  /* 客户端数据接收缓冲区 */
    size_t client_offset = 0;
    const size_t client_frame_bytes = CLIENT_BUFFER_BYTES;
    int total_received = 0;
    int frame_count = 0;

    LOG_INFO("Audio recv thread started");
#if USE_G711A
    LOG_INFO("Client format: 16kHz/mono/G.711a (%zu bytes/frame)",
             client_frame_bytes);
#else
    LOG_INFO("Client format: 16kHz/mono/PCM16LE (%zu bytes/frame)",
             client_frame_bytes);
#endif
    LOG_INFO("Device format: 16kHz/%dch/%dbit (%zu bytes/frame)",
             AUDIO_CHANNELS, AUDIO_BIT_WIDTH, (size_t)AUDIO_BUFFER_BYTES);

    /* 设置 recv 超时为 100ms，避免阻塞导致 g_is_playing 无法更新 */
    struct timeval tv = {0, 100000};  /* 100ms */
    setsockopt(g_client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    /* 正常播放循环 */
    while (g_audio_running && g_client_fd >= 0) {
        /* 检查 ALSA 缓冲区延迟，如果已播放完则清除标志 */
        if (g_is_playing && g_pcm_playback) {
            snd_pcm_sframes_t delay = 0;
            int ret = snd_pcm_delay(g_pcm_playback, &delay);
            if (ret < 0) {
                /* snd_pcm_delay 返回错误（如 underrun 后 -EPIPE），播放已停止 */
                g_is_playing = 0;
            } else if (delay <= 0) {
                g_is_playing = 0;  /* 缓冲区已空，播放结束 */
            }
        }
        
        /* 从客户端接收数据 */
        size_t space = client_frame_bytes - client_offset;
        ssize_t received = recv(g_client_fd, client_buf + client_offset, space, 0);
        if (received <= 0) {
            if (received < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    /* recv 超时，客户端暂时没有数据 */
                    continue;  /* 继续循环检查 g_is_playing */
                }
                LOG_ERR("Recv audio failed: %s", strerror(errno));
            } else {
                LOG_INFO("Client disconnected");
            }
            break;
        }

        total_received += received;
        client_offset += received;

        /* 未凑满一帧数据，继续接收 */
        if (client_offset < client_frame_bytes) {
            continue;
        }

        /* 满帧：解码为单声道 PCM，再转换为立体声 */
        int16_t *stereo_samples = (int16_t *)stereo_buf;
        
#if USE_G711A
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            int16_t sample = alaw_to_pcm(client_buf[i]);
            stereo_samples[i * 2]     = sample;  /* 左声道 */
            stereo_samples[i * 2 + 1] = sample;  /* 右声道 */
        }
#else
        int16_t *mono = (int16_t *)client_buf;
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            stereo_samples[i * 2]     = mono[i];  /* 左声道 */
            stereo_samples[i * 2 + 1] = mono[i];  /* 右声道 */
        }
#endif

        /* 写入 ALSA 播放立体声 */
        frame_count++;

        snd_pcm_sframes_t written = snd_pcm_writei(g_pcm_playback,
                                     stereo_buf, AUDIO_FRAME_SAMPLES);
        if (written < 0) {
            if (written == -EPIPE) {
                /* underrun 发生，使用 snd_pcm_recover 恢复 */
                int recover_ret = snd_pcm_recover(g_pcm_playback, written, 1);
                if (recover_ret < 0) {
                    LOG_ERR("Playback recover failed: %s", snd_strerror(recover_ret));
                    break;
                }
                LOG_WARN("Playback underrun recovered (frame #%d), retry write", frame_count);
                /* 重试写入当前帧 */
                written = snd_pcm_writei(g_pcm_playback, stereo_buf, AUDIO_FRAME_SAMPLES);
                if (written < 0) {
                    LOG_ERR("Playback write after recover failed: %s", snd_strerror(written));
                    break;
                }
            } else {
                LOG_WARN("Playback write failed: %s", snd_strerror(written));
            }
        }
        
        /* 写入成功后才标记为播放中（ALSA 缓冲区有数据在播放） */
        g_is_playing = 1;

        client_offset = 0;
    }

    LOG_INFO("Audio recv thread exited (total received: %d bytes, %d frames, avg: %d bytes/frame)",
             total_received, frame_count,
             frame_count > 0 ? total_received / frame_count : 0);
    g_is_playing = 0;
    return NULL;
}

/* ======================== 客户端连接处理 ======================== */

static void handle_client(int client_fd) {
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    pthread_t send_tid, recv_tid;

    g_client_fd = client_fd;
    g_is_playing = 0;

    getpeername(client_fd, (struct sockaddr *)&client_addr, &addr_len);
    LOG_INFO("Client connected: %s:%d",
             inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    /* Lazy enable audio on client connect - saves CPU when idle */
    if (audio_capture_enable() != 0) {
        LOG_ERR("Failed to enable audio capture");
        close(client_fd);
        g_client_fd = -1;
        return;
    }
    if (audio_playback_enable() != 0) {
        LOG_ERR("Failed to enable audio playback");
        audio_capture_disable();
        close(client_fd);
        g_client_fd = -1;
        return;
    }

    spk_ctrl_enable();

    if (pthread_create(&send_tid, NULL, audio_send_thread, NULL) != 0) {
        LOG_ERR("Failed to create send thread");
        audio_capture_disable();
        audio_playback_disable();
        close(client_fd);
        g_client_fd = -1;
        return;
    }

    if (pthread_create(&recv_tid, NULL, audio_recv_thread, NULL) != 0) {
        LOG_ERR("Failed to create recv thread");
        g_audio_running = 0;
        pthread_join(send_tid, NULL);
        audio_capture_disable();
        audio_playback_disable();
        close(client_fd);
        g_client_fd = -1;
        return;
    }

    pthread_join(send_tid, NULL);
    pthread_join(recv_tid, NULL);

    close(client_fd);
    g_client_fd = -1;

    spk_ctrl_disable();

    /* Disable audio on client disconnect - save CPU when idle */
    audio_capture_disable();
    audio_playback_disable();

    LOG_INFO("Client disconnected");
}

/* ======================== TCP 服务器线程 ======================== */

static void *audio_server_thread(void *arg) {
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    LOG_INFO("Audio server thread started, listening on port %d", TCP_AUDIO_PORT);

    while (g_audio_running) {
        int client_fd = accept(g_server_fd, (struct sockaddr *)&client_addr, &addr_len);

        if (client_fd < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  /* accept 超时，检查 g_audio_running */
            }
            if (g_audio_running) {
                LOG_ERR("Accept failed: %s", strerror(errno));
            }
            break;
        }

        handle_client(client_fd);

        if (!g_audio_running) {
            break;
        }

        LOG_INFO("Waiting for next client connection...");
    }

    LOG_INFO("Audio server thread exited");
    return NULL;
}

/* ======================== 公开接口 ======================== */

int audio_init(void) {
    struct sockaddr_in server_addr;
    int opt = 1;

    Printf("[Audio] Initializing audio module (ALSA)...\n");
    Printf("[Audio] HW format: %dHz/%dch/%dbit\n", AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, AUDIO_BIT_WIDTH);
#if USE_G711A
    Printf("[Audio] Network format: %dHz/mono/G.711a (%d bytes/frame)\n", AUDIO_SAMPLE_RATE, CLIENT_BUFFER_BYTES);
#else
    Printf("[Audio] Network format: %dHz/mono/PCM16LE (%d bytes/frame)\n", AUDIO_SAMPLE_RATE, CLIENT_BUFFER_BYTES);
#endif
    Printf("[Audio] TCP port: %d\n", TCP_AUDIO_PORT);

    /* 初始化 ALSA 采集 */
    if (audio_capture_init() != 0) {
        LOG_ERR("Failed to initialize ALSA capture");
        return -1;
    }

    /* 初始化 ALSA 播放 */
    if (audio_playback_init() != 0) {
        LOG_ERR("Failed to initialize ALSA playback");
        goto exit_capture;
    }

    /* 创建 TCP 服务器 socket */
    g_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_server_fd < 0) {
        LOG_ERR("Socket creation failed: %s", strerror(errno));
        goto exit_playback;
    }

    setsockopt(g_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* accept 超时 1s，确保 audio_stop 能及时退出 */
    struct timeval accept_tv = {1, 0};
    setsockopt(g_server_fd, SOL_SOCKET, SO_RCVTIMEO, &accept_tv, sizeof(accept_tv));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_AUDIO_PORT);

    if (bind(g_server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        LOG_ERR("Bind failed: %s", strerror(errno));
        goto exit_socket;
    }

    if (listen(g_server_fd, 5) < 0) {
        LOG_ERR("Listen failed: %s", strerror(errno));
        goto exit_socket;
    }

    LOG_INFO("Audio module initialized successfully");
    return 0;

exit_socket:
    close(g_server_fd);
    g_server_fd = -1;
exit_playback:
    if (g_pcm_playback) {
        snd_pcm_close(g_pcm_playback);
        g_pcm_playback = NULL;
    }
exit_capture:
    if (g_pcm_capture) {
        snd_pcm_close(g_pcm_capture);
        g_pcm_capture = NULL;
    }
    return -1;
}

int audio_start(void) {
    if (g_server_fd < 0) {
        LOG_ERR("Audio not initialized, call audio_init() first");
        return -1;
    }

    if (g_audio_running) {
        LOG_WARN("Audio already running");
        return 0;
    }

    g_audio_running = 1;

    if (pthread_create(&g_audio_thread_id, NULL, audio_server_thread, NULL) != 0) {
        LOG_ERR("Failed to create audio server thread");
        g_audio_running = 0;
        return -1;
    }

    LOG_INFO("Audio service started");
    return 0;
}

void audio_stop(void) {
    if (!g_audio_running) {
        return;
    }

    LOG_INFO("Stopping audio service...");

    /* 先关闭 fd，唤醒阻塞的 accept/recv */
    if (g_client_fd >= 0) {
        shutdown(g_client_fd, SHUT_RDWR);  /* 立即中断 recv */
        close(g_client_fd);
        g_client_fd = -1;
    }

    if (g_server_fd >= 0) {
        close(g_server_fd);  /* 立即中断 accept */
        g_server_fd = -1;
    }

    /* 再设置退出标志 */
    g_audio_running = 0;

    if (g_audio_thread_id != 0) {
        pthread_join(g_audio_thread_id, NULL);
        g_audio_thread_id = 0;
    }

    LOG_INFO("Audio service stopped");
}

void audio_cleanup(void) {
    LOG_INFO("Cleaning up audio module...");

    audio_stop();

    if (g_pcm_capture) {
        snd_pcm_close(g_pcm_capture);
        g_pcm_capture = NULL;
    }

    if (g_pcm_playback) {
        snd_pcm_close(g_pcm_playback);
        g_pcm_playback = NULL;
    }

    spk_ctrl_disable();

    LOG_INFO("Audio module cleanup done");
}
