/*
 * audio_rk.c - 双向对讲音频模块（Rockchip MPI 版本）
 *
 * 功能：
 * - 监听 TCP 5102 端口
 * - 接收客户端连接后，发送麦克风采集的音频流
 * - 接收客户端的音频流，解码后播放
 * - 可选 VQE(AEC+ANR) 硬件回声消除，支持全双工对讲（ENABLE_VQE=1）
 * - 音频收发在独立线程处理
 *
 * 本文件以已验证正常工作的 ALSA 版(audio.c)为蓝本重构，
 * 保持相同的帧处理架构（一次 GetFrame = 一帧 320 采样/声道，
 * 取左声道 → A-law → 发送），底层用 Rockchip MPI 替换 ALSA。
 *
 * 音频格式：
 *   硬件侧：16kHz / 立体声(硬件要求) / 16bit PCM
 *   网络侧：USE_G711A=1 时 16kHz/mono/G.711a (320 bytes/frame)
 *            USE_G711A=0 时 16kHz/mono/PCM16LE (640 bytes/frame)
 * 底层接口：Rockchip MPI (rk_mpi_ai / rk_mpi_ao)
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
#include <fcntl.h>

/* Rockchip MPI 音频 API */
#include <rk_mpi_ai.h>
#include <rk_mpi_ao.h>
#include <rk_mpi_mb.h>
#include <rk_mpi_sys.h>
#include <rk_mpi_amix.h>
#include "log.h"

/* ======================== 配置参数 ======================== */

#define TCP_AUDIO_PORT          5102
#define AUDIO_SAMPLE_RATE       16000   /* 16kHz */
#define AUDIO_CHANNELS          2       /* 立体声 (RV1103 声卡硬件要求必须为 2 声道) */
#define AUDIO_BIT_WIDTH         16      /* 16bit */
#define AUDIO_FRAME_SAMPLES     320     /* 每帧采样点数 (20ms @ 16kHz)，每声道 */
#define AUDIO_BUFFER_BYTES      (AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS * (AUDIO_BIT_WIDTH / 8))

/* 客户端音频格式 */
#define CLIENT_AUDIO_CHANNELS   1       /* 客户端发送单声道 */
#define USE_G711A               1       /* 1=G.711a编码(320B/帧), 0=原始PCM16LE(640B/帧) —— 手机端用 A-law，必须为 1 */

/* VQE(AEC+ANR) 开关：0=关闭(基础对讲，半双工靠上层)，1=开启硬件回声消除(全双工)。
 * 先用 0 验证基础收发正常，再置 1 测试 AEC。*/
#define ENABLE_VQE              1

#if USE_G711A
#define CLIENT_BYTES_PER_SAMPLE 1       /* G.711a: 每个采样点 1 byte */
#else
#define CLIENT_BYTES_PER_SAMPLE 2       /* 原始 PCM: 每个采样点 2 bytes (16bit) */
#endif
#define CLIENT_BUFFER_BYTES     (AUDIO_FRAME_SAMPLES * CLIENT_AUDIO_CHANNELS * CLIENT_BYTES_PER_SAMPLE)

/* AI/AO 设备和通道 */
#define AI_DEV_ID               0
#define AI_CHN_ID               0
#define AO_DEV_ID               0
#define AO_CHN_ID               0

/* VQE 配置文件路径（含 AEC + ANR） */
#define VQE_CONFIG_PATH         "/oem/usr/share/vqefiles/config_aivqe.json"

/* 功放控制 GPIO */
#define SPK_CTRL_GPIO           59      /* 功放控制引脚，高电平有效 */
#define __SPK_CTRL_GPIO_STR     "59"    /* 字符串形式 */

/* ======================== 日志宏 ======================== */

#define LOG_INFO(fmt, ...)  Printf("[Audio][I] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  Printf("[Audio][W] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)   Printf("[Audio][E] " fmt "\n", ##__VA_ARGS__)

/* ======================== G.711a (A-law) 编解码 ======================== */

#if USE_G711A

/* G.711a 编码：16-bit 线性 PCM → 8-bit A-law (ITU-T G.711)
 * 与 audio.c 一致的实现，往返 SNR ≈ 34dB。*/
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

    if (pcm < 256) {
        seg = 0;
    } else {
        seg = 1;
        while (seg < 7 && pcm >= (256 << seg)) seg++;
    }

    int mantissa;
    if (seg == 0) {
        mantissa = (pcm >> 4) & 0x0F;
    } else {
        mantissa = (pcm >> (seg + 3)) & 0x0F;
    }

    int aval = (seg << 4) | mantissa;
    return (uint8_t)(sign | (aval ^ 0x55));
}

/* G.711a 解码：8-bit A-law → 16-bit 线性 PCM (ITU-T G.711) */
static int16_t alaw_to_pcm(uint8_t alaw_val) {
    int sign, seg, mantissa;
    int pcm;

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
#if !ENABLE_VQE
static volatile int g_is_playing = 0;   /* 播放中标识，无 VQE 时 send 线程据此暂停发送避免回声 */
#endif
static volatile int g_ai_enabled = 0;
static volatile int g_ao_enabled = 0;
static int g_client_fd = -1;
static int g_server_fd = -1;
static pthread_t g_audio_thread_id = 0;

/* ======================== AI/AO 增益配置（对齐 audio.c 的 amixer 设置） ======================== */

static void audio_setup_mixer(void) {
    /* ADC 差分模式（板载差分 ECM，内核已强制差分，此处双保险）。
     * RV1103 差分仅支持 DiffadcL。必须在 AI_Enable 之前设置。*/
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "ADC Mode", "DiffadcL");
    /* 增益：与 audio.c 一致 */
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "ADC ALC Left Volume", "26");
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "ADC ALC Right Volume", "26");
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "ADC MIC Left Gain", "3");
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "ADC MIC Right Gain", "3");
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "ADC MICBIAS Voltage", "VREFx0_975");
    LOG_INFO("Mixer configured (DiffadcL, ALC=26, MIC Gain=3, MICBIAS=0.975)");
}

/* ======================== AI (采集) 初始化 ======================== */

static int audio_capture_init(void) {
    int ret;
    AIO_ATTR_S ai_attr;

    spk_ctrl_init();

#ifndef STANDALONE_TEST
    ret = RK_MPI_SYS_Init();
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_SYS_Init failed: 0x%x", ret);
        return -1;
    }
#endif

    /* 清理残留资源 */
    RK_MPI_AI_DisableVqe(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_DisableChn(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_Disable(AI_DEV_ID);

    memset(&ai_attr, 0, sizeof(ai_attr));
    ai_attr.soundCard.channels = AUDIO_CHANNELS;
    ai_attr.soundCard.sampleRate = AUDIO_SAMPLE_RATE;
    ai_attr.soundCard.bitWidth = AUDIO_BIT_WIDTH_16;
    ai_attr.enSamplerate = AUDIO_SAMPLE_RATE_16000;
    ai_attr.enBitwidth = AUDIO_BIT_WIDTH_16;
    ai_attr.enSoundmode = AUDIO_SOUND_MODE_STEREO;  /* 与 channels=2 一致，避免帧长/声道语义混乱 */
    ai_attr.u32FrmNum = 4;
    /* u32PtNumPerFrm 经实测是“每帧总样本数(跨所有声道)”，且 GetFrame 实际
     * 返回的每声道采样数 = PtNumPerFrm/4。要每帧 320 采样/声道(20ms@16k)，
     * 需设为 320*4 = 1280。此值下实测 frame_gap=20ms、速率=16KB/s，节奏正确。*/
    ai_attr.u32PtNumPerFrm = AUDIO_FRAME_SAMPLES * 4;  /* =1280 → 320 采样/声道/帧 */
    ai_attr.u32EXFlag = 0;
    ai_attr.u32ChnCnt = AUDIO_CHANNELS;
    snprintf((char*)ai_attr.u8CardName, sizeof(ai_attr.u8CardName), "hw:0,0");

    ret = RK_MPI_AI_SetPubAttr(AI_DEV_ID, &ai_attr);
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_AI_SetPubAttr failed: 0x%x", ret);
        return -1;
    }

    LOG_INFO("AI attributes set (lazy enable on client connect)");
    return 0;
}

/* Enable AI - lazy startup on client connect */
static int audio_capture_enable(void) {
    int ret;

    if (g_ai_enabled) {
        return 0;
    }

    /* 增益/模式必须在 Enable 之前设置 */
    audio_setup_mixer();

#if ENABLE_VQE
    /* AEC 回采：把 AO 播放信号回采到采集右声道作为参考。仅 VQE 需要。*/
    ret = RK_MPI_AMIX_SetControl(AI_DEV_ID, "I2STDM Digital Loopback Mode", "Mode2");
    if (ret != RK_SUCCESS) {
        LOG_WARN("I2STDM Loopback Mode2 set failed: 0x%x", ret);
    } else {
        LOG_INFO("I2STDM Loopback Mode2 enabled (AEC reference)");
    }
#else
    /* 关闭回采，确保右声道不引入播放信号 */
    RK_MPI_AMIX_SetControl(AI_DEV_ID, "I2STDM Digital Loopback Mode", "Disabled");
#endif

    ret = RK_MPI_AI_Enable(AI_DEV_ID);
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_AI_Enable failed: 0x%x", ret);
        return -1;
    }

    AI_CHN_PARAM_S chn_param;
    memset(&chn_param, 0, sizeof(chn_param));
    chn_param.s32UsrFrmDepth = 4;
    ret = RK_MPI_AI_SetChnParam(AI_DEV_ID, AI_CHN_ID, &chn_param);
    if (ret != RK_SUCCESS) {
        LOG_WARN("RK_MPI_AI_SetChnParam failed: 0x%x", ret);
    }

#if ENABLE_VQE
    /* VQE 配置必须在 EnableChn 之前 */
    AI_VQE_CONFIG_S vqe_config;
    memset(&vqe_config, 0, sizeof(vqe_config));
    vqe_config.enCfgMode = AIO_VQE_CONFIG_LOAD_FILE;
    memcpy(vqe_config.aCfgFile, VQE_CONFIG_PATH, strlen(VQE_CONFIG_PATH) + 1);
    vqe_config.s32WorkSampleRate = AUDIO_SAMPLE_RATE;
    vqe_config.s32FrameSample = 160;  /* VQE 仅支持 256(16ms)/160(10ms) */
    ret = RK_MPI_AI_SetVqeAttr(AI_DEV_ID, AI_CHN_ID, AO_DEV_ID, AO_CHN_ID, &vqe_config);
    if (ret != RK_SUCCESS) {
        LOG_WARN("RK_MPI_AI_SetVqeAttr failed: 0x%x", ret);
    } else {
        ret = RK_MPI_AI_EnableVqe(AI_DEV_ID, AI_CHN_ID);
        if (ret != RK_SUCCESS) {
            LOG_WARN("RK_MPI_AI_EnableVqe failed: 0x%x", ret);
        } else {
            LOG_INFO("VQE enabled: AEC + ANR");
        }
    }
#else
    LOG_INFO("VQE disabled (ENABLE_VQE=0)");
#endif

    ret = RK_MPI_AI_EnableChn(AI_DEV_ID, AI_CHN_ID);
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_AI_EnableChn failed: 0x%x", ret);
        return -1;
    }

    RK_MPI_AI_SetVolume(AI_DEV_ID, 100);
    RK_MPI_AI_SetTrackMode(AI_DEV_ID, AUDIO_TRACK_NORMAL);

    g_ai_enabled = 1;
    LOG_INFO("=== AI enabled ===");
    return 0;
}

static void audio_capture_disable(void) {
    if (!g_ai_enabled) {
        return;
    }
    RK_MPI_AI_DisableVqe(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_DisableChn(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_Disable(AI_DEV_ID);
    g_ai_enabled = 0;
    LOG_INFO("=== AI disabled ===");
}

/* ======================== AO (播放) 初始化 ======================== */

static int audio_playback_init(void) {
    int ret;
    AIO_ATTR_S ao_attr;

    RK_MPI_AO_DisableChn(AO_DEV_ID, AO_CHN_ID);
    RK_MPI_AO_Disable(AO_DEV_ID);

    memset(&ao_attr, 0, sizeof(ao_attr));
    ao_attr.soundCard.channels = AUDIO_CHANNELS;
    ao_attr.soundCard.sampleRate = AUDIO_SAMPLE_RATE;
    ao_attr.soundCard.bitWidth = AUDIO_BIT_WIDTH_16;
    ao_attr.enSamplerate = AUDIO_SAMPLE_RATE_16000;
    ao_attr.enBitwidth = AUDIO_BIT_WIDTH_16;
    ao_attr.enSoundmode = AUDIO_SOUND_MODE_STEREO;
    ao_attr.u32FrmNum = 4;
    ao_attr.u32PtNumPerFrm = AUDIO_FRAME_SAMPLES;
    ao_attr.u32EXFlag = 0;
    ao_attr.u32ChnCnt = AUDIO_CHANNELS;
    snprintf((char*)ao_attr.u8CardName, sizeof(ao_attr.u8CardName), "hw:0,0");

    ret = RK_MPI_AO_SetPubAttr(AO_DEV_ID, &ao_attr);
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_AO_SetPubAttr failed: 0x%x", ret);
        return -1;
    }
    LOG_INFO("AO attributes set (lazy enable on client connect)");
    return 0;
}

static int audio_playback_enable(void) {
    int ret;

    if (g_ao_enabled) {
        return 0;
    }

    ret = RK_MPI_AO_Enable(AO_DEV_ID);
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_AO_Enable failed: 0x%x", ret);
        return -1;
    }
    ret = RK_MPI_AO_EnableChn(AO_DEV_ID, AO_CHN_ID);
    if (ret != RK_SUCCESS) {
        LOG_ERR("RK_MPI_AO_EnableChn failed: 0x%x", ret);
        RK_MPI_AO_Disable(AO_DEV_ID);
        return -1;
    }

    g_ao_enabled = 1;
    LOG_INFO("=== AO enabled ===");
    return 0;
}

static void audio_playback_disable(void) {
    if (!g_ao_enabled) {
        return;
    }
    RK_MPI_AO_DisableChn(AO_DEV_ID, AO_CHN_ID);
    RK_MPI_AO_Disable(AO_DEV_ID);
    g_ao_enabled = 0;
    LOG_INFO("=== AO disabled ===");
}

/* ======================== 发送线程（采集 → 客户端） ======================== */

static void *audio_send_thread(void *arg) {
    /* mono 累积缓冲：兼容不同帧长/声道模式（非VQE=STEREO 320/声道，VQE=MONO 帧长可能不同），
     * 累积满一个网络帧(AUDIO_FRAME_SAMPLES) 就编码发送。预留 2 倍空间。*/
    int16_t mono_acc[AUDIO_FRAME_SAMPLES * 4];
    int acc_fill = 0;
#if USE_G711A
    uint8_t client_buf[CLIENT_BUFFER_BYTES];
#else
    uint8_t net_pcm[AUDIO_FRAME_SAMPLES];
    uint8_t *client_buf = (uint8_t *)net_pcm;
#endif
    AUDIO_FRAME_S frame;
    int ret;

    LOG_INFO("Audio send thread started");

    while (g_audio_running && g_client_fd >= 0) {
        /* 阻塞获取一帧（timeout=-1 阻塞，节奏由硬件出帧驱动，等价 ALSA snd_pcm_readi）*/
        ret = RK_MPI_AI_GetFrame(AI_DEV_ID, AI_CHN_ID, &frame, NULL, -1);
        if (ret != RK_SUCCESS) {
            if (!g_audio_running) break;
            continue;
        }

        void *data = RK_MPI_MB_Handle2VirAddr(frame.pMbBlk);
        if (!data) {
            RK_MPI_AI_ReleaseFrame(AI_DEV_ID, AI_CHN_ID, &frame, NULL);
            continue;
        }

#if !ENABLE_VQE
        /* 无 VQE：播放中暂停发送，避免回声（半双工，同 audio.c）*/
        if (g_is_playing) {
            RK_MPI_AI_ReleaseFrame(AI_DEV_ID, AI_CHN_ID, &frame, NULL);
            continue;
        }
#endif

        /* 提取单声道 mic 采样：
         *   MONO(VQE 输出) → 连续取 samples[i]
         *   STEREO(原始采集) → 交织取左声道 samples[i*channels]
         * u32Len 为整帧总字节数。*/
        int16_t *samples = (int16_t *)data;
        int total_i16 = (int)(frame.u32Len / sizeof(int16_t));
        int is_stereo = (frame.enSoundMode == AUDIO_SOUND_MODE_STEREO);
        int n = is_stereo ? (total_i16 / AUDIO_CHANNELS) : total_i16;
        for (int i = 0; i < n && acc_fill < (int)(sizeof(mono_acc)/sizeof(int16_t)); i++) {
            mono_acc[acc_fill++] = is_stereo ? samples[i * AUDIO_CHANNELS] : samples[i];
        }

        RK_MPI_AI_ReleaseFrame(AI_DEV_ID, AI_CHN_ID, &frame, NULL);

        /* 累积满一个网络帧就发送（一帧输入可能产生 0/1/多个网络帧）*/
        while (acc_fill >= AUDIO_FRAME_SAMPLES) {
#if USE_G711A
            for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
                client_buf[i] = pcm_to_alaw(mono_acc[i]);
            }
#else
            memcpy(client_buf, mono_acc, AUDIO_FRAME_SAMPLES * sizeof(int16_t));
#endif
            ssize_t sent = send(g_client_fd, client_buf, CLIENT_BUFFER_BYTES, MSG_NOSIGNAL);
            if (sent < 0) {
                LOG_ERR("Send audio failed: %s", strerror(errno));
                goto send_exit;
            } else if ((size_t)sent != CLIENT_BUFFER_BYTES) {
                LOG_WARN("Partial send: %zd/%d bytes", sent, CLIENT_BUFFER_BYTES);
            }
            acc_fill -= AUDIO_FRAME_SAMPLES;
            if (acc_fill > 0) {
                memmove(mono_acc, mono_acc + AUDIO_FRAME_SAMPLES, acc_fill * sizeof(int16_t));
            }
        }
    }
send_exit:

    LOG_INFO("Audio send thread exited");
    return NULL;
}

/* ======================== 接收线程（客户端 → 播放） ======================== */

static void *audio_recv_thread(void *arg) {
    char stereo_buf[AUDIO_BUFFER_BYTES];
    uint8_t client_buf[CLIENT_BUFFER_BYTES];
    size_t client_offset = 0;
    const size_t client_frame_bytes = CLIENT_BUFFER_BYTES;
    int total_received = 0;
    int frame_count = 0;
    int ret;

    LOG_INFO("Audio recv thread started");

    /* recv 超时 100ms，便于及时更新 g_is_playing / 检查退出 */
    struct timeval tv = {0, 100000};
    setsockopt(g_client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (g_audio_running && g_client_fd >= 0) {
        size_t space = client_frame_bytes - client_offset;
        ssize_t received = recv(g_client_fd, client_buf + client_offset, space, 0);
        if (received <= 0) {
            if (received < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                LOG_ERR("Recv audio failed: %s", strerror(errno));
            } else {
                LOG_INFO("Client disconnected");
            }
            break;
        }

        total_received += received;
        client_offset += received;
        if (client_offset < client_frame_bytes) {
            continue;
        }

        /* 满帧：解码为单声道 → 复制成立体声 */
        int16_t *stereo_samples = (int16_t *)stereo_buf;
#if USE_G711A
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            int16_t s = alaw_to_pcm(client_buf[i]);
            stereo_samples[i * 2]     = s;
            stereo_samples[i * 2 + 1] = s;
        }
#else
        int16_t *mono = (int16_t *)client_buf;
        for (int i = 0; i < AUDIO_FRAME_SAMPLES; i++) {
            stereo_samples[i * 2]     = mono[i];
            stereo_samples[i * 2 + 1] = mono[i];
        }
#endif

        /* 送 AO 播放（拷贝模式，避免 DMA 竞争）*/
        AUDIO_FRAME_S frame;
        memset(&frame, 0, sizeof(frame));
        MB_EXT_CONFIG_S ext;
        memset(&ext, 0, sizeof(ext));
        ext.pOpaque = stereo_buf;
        ext.pu8VirAddr = (RK_U8 *)stereo_buf;
        ext.u64Size = AUDIO_BUFFER_BYTES;

        ret = RK_MPI_SYS_CreateMB(&frame.pMbBlk, &ext);
        if (ret != RK_SUCCESS || !frame.pMbBlk) {
            LOG_ERR("CreateMB failed: 0x%x", ret);
            client_offset = 0;
            continue;
        }
        frame.enBitWidth = AUDIO_BIT_WIDTH_16;
        frame.enSoundMode = AUDIO_SOUND_MODE_STEREO;
        frame.u32Len = AUDIO_BUFFER_BYTES;
        frame.s32SampleRate = AUDIO_SAMPLE_RATE;
        frame.bBypassMbBlk = RK_FALSE;

        frame_count++;
        ret = RK_MPI_AO_SendFrame(AO_DEV_ID, AO_CHN_ID, &frame, 1000);
        if (ret != RK_SUCCESS) {
            LOG_WARN("AO_SendFrame failed: 0x%x", ret);
        }
        RK_MPI_MB_ReleaseMB(frame.pMbBlk);

#if !ENABLE_VQE
        g_is_playing = 1;  /* 标记播放中，send 线程暂停发送避免回声 */
#endif

        client_offset = 0;
    }

    LOG_INFO("Audio recv thread exited (%d bytes, %d frames)", total_received, frame_count);
#if !ENABLE_VQE
    g_is_playing = 0;
#endif
    return NULL;
}

#if !ENABLE_VQE
/* 播放状态监控线程：AO 缓冲播完后清除 g_is_playing，恢复采集发送。
 * 用 AO 上报的缓冲延迟判断是否播放完毕。*/
static void *audio_playstate_thread(void *arg) {
    while (g_audio_running && g_client_fd >= 0) {
        if (g_is_playing) {
            AO_CHN_STATE_S state;
            memset(&state, 0, sizeof(state));
            if (RK_MPI_AO_QueryChnStat(AO_DEV_ID, AO_CHN_ID, &state) == RK_SUCCESS) {
                /* 缓冲区中待播放帧数为 0 → 播放完毕 */
                if (state.u32ChnBusyNum == 0) {
                    g_is_playing = 0;
                }
            } else {
                g_is_playing = 0;
            }
        }
        usleep(20000);  /* 20ms 轮询 */
    }
    return NULL;
}
#endif

/* ======================== 客户端连接处理 ======================== */

static void handle_client(int client_fd) {
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    pthread_t send_tid, recv_tid;
#if !ENABLE_VQE
    pthread_t state_tid;
    int state_ok = 0;
#endif

    g_client_fd = client_fd;
#if !ENABLE_VQE
    g_is_playing = 0;
#endif

    getpeername(client_fd, (struct sockaddr *)&client_addr, &addr_len);
    LOG_INFO("Client connected: %s:%d",
             inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

    if (audio_capture_enable() != 0) {
        LOG_ERR("Failed to enable AI");
        close(client_fd);
        g_client_fd = -1;
        return;
    }
    if (audio_playback_enable() != 0) {
        LOG_ERR("Failed to enable AO");
        audio_capture_disable();
        close(client_fd);
        g_client_fd = -1;
        return;
    }

    spk_ctrl_enable();

    if (pthread_create(&send_tid, NULL, audio_send_thread, NULL) != 0) {
        LOG_ERR("Failed to create send thread");
        goto cleanup;
    }
    if (pthread_create(&recv_tid, NULL, audio_recv_thread, NULL) != 0) {
        LOG_ERR("Failed to create recv thread");
        g_audio_running = 0;
        pthread_join(send_tid, NULL);
        goto cleanup;
    }
#if !ENABLE_VQE
    state_ok = (pthread_create(&state_tid, NULL, audio_playstate_thread, NULL) == 0);
#endif

    pthread_join(send_tid, NULL);
    pthread_join(recv_tid, NULL);
#if !ENABLE_VQE
    if (state_ok) pthread_join(state_tid, NULL);
#endif

cleanup:
    close(client_fd);
    g_client_fd = -1;
    spk_ctrl_disable();
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

        if (!g_audio_running) break;
        LOG_INFO("Waiting for next client connection...");
    }

    LOG_INFO("Audio server thread exited");
    return NULL;
}

/* ======================== 公开接口 ======================== */

int audio_init(void) {
    struct sockaddr_in server_addr;
    int opt = 1;

    Printf("[Audio] Initializing audio module (Rockchip MPI)...\n");
    Printf("[Audio] HW format: %dHz/%dch/%dbit\n", AUDIO_SAMPLE_RATE, AUDIO_CHANNELS, AUDIO_BIT_WIDTH);
#if USE_G711A
    Printf("[Audio] Network format: %dHz/mono/G.711a (%d bytes/frame)\n", AUDIO_SAMPLE_RATE, CLIENT_BUFFER_BYTES);
#else
    Printf("[Audio] Network format: %dHz/mono/PCM16LE (%d bytes/frame)\n", AUDIO_SAMPLE_RATE, CLIENT_BUFFER_BYTES);
#endif
    Printf("[Audio] VQE(AEC): %s\n", ENABLE_VQE ? "ON" : "OFF");
    Printf("[Audio] TCP port: %d\n", TCP_AUDIO_PORT);

    if (audio_capture_init() != 0) {
        LOG_ERR("Failed to initialize AI");
        return -1;
    }
    if (audio_playback_init() != 0) {
        LOG_ERR("Failed to initialize AO");
        goto exit_ai;
    }

    g_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_server_fd < 0) {
        LOG_ERR("Socket creation failed: %s", strerror(errno));
        goto exit_ao;
    }

    /* FD_CLOEXEC：防止 fork+exec 子进程(udhcpc/rproxyc)继承监听 socket 导致端口泄漏 */
    {
        int fl = fcntl(g_server_fd, F_GETFD, 0);
        if (fl != -1) fcntl(g_server_fd, F_SETFD, fl | FD_CLOEXEC);
    }

    setsockopt(g_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(g_server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));
#endif

    /* accept 超时 1s，确保 audio_stop 能及时退出 */
    {
        struct timeval accept_tv = {1, 0};
        setsockopt(g_server_fd, SOL_SOCKET, SO_RCVTIMEO, &accept_tv, sizeof(accept_tv));
    }

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

    LOG_INFO("Audio module initialized successfully (Rockchip MPI)");
    return 0;

exit_socket:
    close(g_server_fd);
    g_server_fd = -1;
exit_ao:
    RK_MPI_AO_DisableChn(AO_DEV_ID, AO_CHN_ID);
    RK_MPI_AO_Disable(AO_DEV_ID);
exit_ai:
    RK_MPI_AI_DisableChn(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_Disable(AI_DEV_ID);
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

    if (g_client_fd >= 0) {
        shutdown(g_client_fd, SHUT_RDWR);
        close(g_client_fd);
        g_client_fd = -1;
    }
    if (g_server_fd >= 0) {
        close(g_server_fd);
        g_server_fd = -1;
    }

    g_audio_running = 0;

    /* 禁用 AI/AO 解除 GetFrame 阻塞 */
    RK_MPI_AI_DisableVqe(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_DisableChn(AI_DEV_ID, AI_CHN_ID);
    RK_MPI_AI_Disable(AI_DEV_ID);
    RK_MPI_AO_DisableChn(AO_DEV_ID, AO_CHN_ID);
    RK_MPI_AO_Disable(AO_DEV_ID);

    if (g_audio_thread_id != 0) {
        pthread_join(g_audio_thread_id, NULL);
        g_audio_thread_id = 0;
    }

    LOG_INFO("Audio service stopped");
}

void audio_cleanup(void) {
    LOG_INFO("Cleaning up audio module (Rockchip MPI)...");
    audio_stop();
    spk_ctrl_disable();
    RK_MPI_SYS_Exit();
    LOG_INFO("Audio module cleanup done");
}
