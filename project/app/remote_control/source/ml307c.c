/*
 * ml307c.c - ML307C 4G module driver
 *
 * Provides AT command send/receive, RNDIS dial-up, and GNSS positioning.
 * AT port: /dev/ttyUSB2 (fixed, per ML307C USB interface mapping).
 *
 * NMEA GNRMC data is parsed from the AT port URC stream because ML307C
 * does not support nmea/port=1 (USB GPS port output).
 *
 * DESIGN: Single worker thread handles ALL ML307C operations.
 * - Thread opens port, initializes module, then enters main loop
 * - Main loop: read AT port, parse NMEA/AT responses, handle pending commands
 * - Main thread queues AT commands via shared state
 * - No extra threads, everything in one ML307C worker thread
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>

#include "ml307c.h"

/* ======================== Configuration ======================== */

#define AT_TIMEOUT_MS       5000    /* Default AT response timeout */
#define AT_DIAL_TIMEOUT_MS  30000   /* Dial-up timeout (longer) */
#define RESP_BUF_SIZE       1024
#define LINE_BUF_SIZE       512
#define RNDIS_IFACE         "eth1"

/* ======================== Module State ======================== */

static int g_at_fd = -1;

/* Line parsing buffer (worker thread only) */
static char   g_line_buf[LINE_BUF_SIZE];
static int    g_line_pos;

/* AT command state (shared between main thread and worker thread) */
static pthread_mutex_t g_at_cmd_mtx   = PTHREAD_MUTEX_INITIALIZER;
static char   g_at_cmd_pending[256];  /* Command queued by main thread */
static int    g_at_cmd_has_pending;   /* 1 = command waiting to be sent */
static char   g_at_resp[RESP_BUF_SIZE];
static int    g_at_resp_len;
static int    g_at_resp_ready;        /* 1 = response ready for main thread */

/* URC flags (set by process_line in worker thread) */
static volatile int g_agnss_data_ready;  /* 1 = +MAGNSSDATA: 1 received */

/* Latest GPS data (protected by mutex) */
static ml307c_gps_data_t g_gps;
static pthread_mutex_t   g_gps_mtx    = PTHREAD_MUTEX_INITIALIZER;

/* Latest signal strength (protected by mutex) */
static int16_t  g_signal_dbm = 0;      /* Signal strength in dBm */
static int64_t  g_signal_update_ms = 0; /* Last update timestamp */
static pthread_mutex_t g_signal_mtx = PTHREAD_MUTEX_INITIALIZER;

/* Worker thread */
static pthread_t     g_worker_tid;
static volatile int  g_worker_running;

/* ======================== Utilities ======================== */

static int64_t now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/*
 * Interruptible sleep - checks g_worker_running every 100ms.
 * Returns 0 if sleep completed, -1 if interrupted by shutdown.
 */
static int ml307c_sleep(int ms)
{
    int iterations = ms / 100;
    for (int i = 0; i < iterations && g_worker_running; i++)
        usleep(100000);  /* 100ms */
    return g_worker_running ? 0 : -1;
}

/* ======================== URC Handling ======================== */

/*
 * Wait for specific async URC after AT command returns OK.
 * The URC content is matched in process_line() which sets the flag.
 * Returns 0 if URC received, -1 if timeout or interrupted.
 */
static int wait_for_urc(volatile int *flag, const char *urc_prefix, int timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;
    
    if (urc_prefix)
        printf("[ML307C] waiting for URC: %s...\n", urc_prefix);
    
    while (now_ms() < deadline && g_worker_running) {
        if (*flag) {
            if (urc_prefix)
                printf("[ML307C] URC received: %s\n", urc_prefix);
            return 0;  /* URC received */
        }
        ml307c_sleep(200);  /* Check every 200ms */
    }
    
    if (urc_prefix)
        fprintf(stderr, "[ML307C] URC timeout: %s\n", urc_prefix);
    return -1;  /* Timeout or interrupted */
}

/* ======================== Serial Port ======================== */

static int serial_open(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[ML307C] open %s: %s\n", dev, strerror(errno));
        return -1;
    }

    struct termios t;
    memset(&t, 0, sizeof(t));

    /* Minimal configuration for USB modem */
    /* 8N1: 8 bits, no parity, 1 stop bit */
    t.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    t.c_cflag &= ~CRTSCTS;  /* No hardware flow control */
    t.c_cflag &= ~PARENB;   /* No parity */
    t.c_cflag &= ~CSTOPB;   /* 1 stop bit */

    /* Raw input */
    t.c_iflag = IGNPAR;     /* Ignore parity errors */
    t.c_iflag &= ~(ICRNL | INLCR | IGNCR);  /* Keep CR/LF as-is */

    /* Raw output */
    t.c_oflag = 0;

    /* Raw mode: no canonical processing, no echo */
    t.c_lflag = 0;

    /* Read timeout: 100ms (VTIME = 10 * 0.1s) */
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 1;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &t);

    return fd;
}

static int serial_write(int fd, const char *data, int len)
{
    int total = 0;
    while (total < len) {
        ssize_t n = write(fd, data + total, len - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "[ML307C] write: %s\n", strerror(errno));
            return -1;
        }
        total += n;
    }
    return total;
}

/* ======================== Line Processing ======================== */

/*
 * Process a complete line from AT port (called from worker thread).
 * Lines starting with '$' are NMEA; others are AT responses.
 */
static void process_line(const char *line)
{
    /* NMEA sentences start with '$' */
    if (line[0] == '$') {
        if (strncmp(line, "$GNRMC", 6) == 0) {
            /* Parse RMC and update GPS data */
            extern void ml307c_parse_rmc(const char *sentence);
            ml307c_parse_rmc(line);
        }
        return;
    }

    /* Accumulate AT response if waiting for response */
    if (!g_at_resp_ready) {
        int avail = RESP_BUF_SIZE - g_at_resp_len - 1;
        if (avail > 0) {
            int n = snprintf(g_at_resp + g_at_resp_len, avail + 1,
                             "%s\n", line);
            if (n > avail) n = avail;
            g_at_resp_len += n;
        }

        /* Check for response terminator */
        if (strcmp(line, "OK") == 0 ||
            strncmp(line, "ERROR", 5) == 0 ||
            strncmp(line, "+CME ERROR", 10) == 0) {
            g_at_resp_ready = 1;
        }
    }

    /* Detect AGNSS data URC: +MAGNSSDATA: 1 (success) */
    if (strncmp(line, "+MAGNSSDATA: 1", 14) == 0) {
        g_agnss_data_ready = 1;
    }

    /* Detect GNSS state URC: +MGNSSURC: "state",1 (GNSS enabled) */
    if (strncmp(line, "+MGNSSURC: \"state\",1", 20) == 0) {
        g_agnss_data_ready = 1;  /* Reuse flag for GNSS state */
    }
}

/* ======================== GNRMC Parser ======================== */

/*
 * Parse NMEA RMC sentence and update global GPS data.
 * Format: $GNRMC,hhmmss.ss,A/V,ddmm.mmmm,N/S,dddmm.mmmm,E/W,
 *         spd_kn,cog,ddmmyy,...*cs
 */
void ml307c_parse_rmc(const char *sentence)
{
    // printf("[ML307C] RMC: %s\n", sentence);

    char buf[256];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* Strip checksum */
    char *star = strchr(buf, '*');
    if (star) *star = '\0';

    char *fields[16];
    int nf = 0;
    char *saveptr;
    char *tok = strtok_r(buf, ",", &saveptr);
    while (tok && nf < 16) {
        fields[nf++] = tok;
        tok = strtok_r(NULL, ",", &saveptr);
    }

    /* RMC: need at least 8 fields */
    if (nf < 8) return;

    /* Field 2: A=valid, V=void */
    if (fields[2][0] != 'A') return;

    /* Field 3,4: latitude (DDMM.MMMMM format) */
    if (strlen(fields[3]) < 4) return;
    double lat_raw = strtod(fields[3], NULL);
    int lat_deg = (int)(lat_raw / 100.0);  /* Extract degrees */
    double lat_min = lat_raw - lat_deg * 100.0;  /* Extract minutes */
    double lat = lat_deg + lat_min / 60.0;  /* Convert to decimal degrees */
    if (fields[4][0] == 'S') lat = -lat;

    /* Field 5,6: longitude (DDDMM.MMMMM format) */
    if (strlen(fields[5]) < 5) return;
    double lon_raw = strtod(fields[5], NULL);
    int lon_deg = (int)(lon_raw / 100.0);  /* Extract degrees */
    double lon_min = lon_raw - lon_deg * 100.0;  /* Extract minutes */
    double lon = lon_deg + lon_min / 60.0;  /* Convert to decimal degrees */
    if (fields[6][0] == 'W') lon = -lon;

    /* Field 7: speed in knots -> km/h */
    float spd_kn = strtof(fields[7], NULL);
    float spd_kmh = spd_kn * 1.852f;

    pthread_mutex_lock(&g_gps_mtx);
    g_gps.latitude  = lat;
    g_gps.longitude = lon;
    g_gps.speed_kmh = spd_kmh;
    g_gps.valid     = 1;
    g_gps.update_ms = now_ms();
    pthread_mutex_unlock(&g_gps_mtx);

    // printf("[ML307C] GPS: lat=%.6f lon=%.6f spd=%.1f km/h\n",
    //        lat, lon, spd_kmh);
}

/* ======================== AT Command Engine ======================== */

/*
 * Send AT command from worker thread.
 * This is called internally by the worker thread during initialization
 * or when processing queued commands from main thread.
 */
static int at_send_worker(const char *cmd, int timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;

    /* Flush and reset response buffer */
    tcflush(g_at_fd, TCIFLUSH);
    g_at_resp_len = 0;
    g_at_resp_ready = 0;
    memset(g_at_resp, 0, sizeof(g_at_resp));

    int cmd_len = strlen(cmd);
    /* Print command without trailing \r\n */
    // printf("[ML307C] TX: %s", cmd);
    // if (cmd_len >= 2 && cmd[cmd_len-2] == '\r' && cmd[cmd_len-1] == '\n')
    //     printf("\n");
    // else
    //     printf("\n");
    
    serial_write(g_at_fd, cmd, cmd_len);

    /* Read and process lines until response or timeout */
    char buf[1024];
    g_line_pos = 0;

    while (now_ms() < deadline && g_worker_running) {
        ssize_t n = read(g_at_fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* No data available, short sleep to check shutdown flag */
                usleep(5000);  /* 5ms - check shutdown quickly */
                continue;
            }
            if (errno == EINTR) continue;
            fprintf(stderr, "[ML307C] read error: %s\n", strerror(errno));
            return -1;
        }
        if (n == 0) {
            usleep(5000);  /* 5ms */
            continue;
        }

        /* Process data line by line */
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\r' || c == '\n') {
                if (g_line_pos > 0) {
                    g_line_buf[g_line_pos] = '\0';
                    process_line(g_line_buf);
                    g_line_pos = 0;
                }
            } else if (g_line_pos < (int)sizeof(g_line_buf) - 1) {
                g_line_buf[g_line_pos++] = c;
            }
        }

        if (g_at_resp_ready)
            break;
    }

    int len = g_at_resp_len;
    if (!g_worker_running) {
        printf("[ML307C] AT interrupted by shutdown\n");
        return -1;
    } else if (g_at_resp_ready) {
        // printf("[ML307C] RX: %s", g_at_resp);
    } else {
        fprintf(stderr, "[ML307C] timeout: %.*s\n", cmd_len - 2, cmd);
    }

    return g_at_resp_ready ? len : -1;
}

/* Check if AT response contains OK */
static int resp_has_ok(const char *resp)
{
    return strstr(resp, "OK") != NULL;
}

/* Send AT and return 0 on OK, -1 on error/timeout */
static int at_ok_worker(const char *cmd, int timeout_ms)
{
    int n = at_send_worker(cmd, timeout_ms);
    if (n < 0) return -1;
    return resp_has_ok(g_at_resp) ? 0 : -1;
}

/* ======================== RNDIS Dial-Up ======================== */

static int wait_module_ready(void)
{
    for (int i = 0; i < 10 && g_worker_running; i++) {
        if (at_ok_worker("AT\r\n", 2000) == 0)
            return 0;
        printf("[ML307C] waiting for module... (%d/10)\n", i + 1);
        if (ml307c_sleep(2000) < 0) return -1;
    }
    if (!g_worker_running)
        return -1;
    fprintf(stderr, "[ML307C] module not responding\n");
    return -1;
}

static int wait_network(int timeout_sec)
{
    int64_t deadline = now_ms() + timeout_sec * 1000;
    while (now_ms() < deadline && g_worker_running) {
        int n = at_send_worker("AT+COPS?\r\n", AT_TIMEOUT_MS);
        if (n > 0 && strstr(g_at_resp, "+COPS: 0")) {
            printf("[ML307C] network registered\n");
            return 0;
        }
        printf("[ML307C] waiting for network...\n");
        if (ml307c_sleep(3000) < 0) return -1;
    }
    fprintf(stderr, "[ML307C] network registration timeout\n");
    return -1;
}

static int rndis_dial(void)
{
    if (at_ok_worker("AT+MDIALUPCFG=\"mode\",0\r\n", AT_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[ML307C] set RNDIS mode failed\n");
        return -1;
    }

    int n = at_send_worker("AT+MDIALUP=1,1\r\n", AT_DIAL_TIMEOUT_MS);
    if (n < 0 || !resp_has_ok(g_at_resp)) {
        fprintf(stderr, "[ML307C] dial failed\n");
        return -1;
    }

    if (strstr(g_at_resp, "+MDIALUP:"))
        printf("[ML307C] RNDIS dial success\n");
    else
        printf("[ML307C] dial OK, awaiting IP...\n");

    printf("[ML307C] running udhcpc on %s...\n", RNDIS_IFACE);
    /* Kill existing udhcpc processes to avoid duplicates */
    system("killall udhcpc 2>/dev/null");
    ml307c_sleep(500);  /* Wait for old processes to exit */
    
    int ret = system("udhcpc -i " RNDIS_IFACE " -b -t 10 -T 3 2>/dev/null");
    if (ret != 0)
        fprintf(stderr, "[ML307C] DHCP failed (non-fatal)\n");

    return 0;
}

/* ======================== GNSS Configuration ======================== */

static int gnss_configure(void)
{
    /* Set NMEA output port to AT port (0 = AT port) */
    if (at_ok_worker("AT+MGNSSCFG=\"nmea/port\",0\r\n", AT_TIMEOUT_MS) < 0)
        fprintf(stderr, "[ML307C] set nmea/port failed\n");

    /* Enable only RMC (bit3 = 0x08 = 8) to reduce data volume */
    if (at_ok_worker("AT+MGNSSCFG=\"nmea/mask\",8\r\n", AT_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[ML307C] set nmea/mask failed\n");
        return -1;
    }

    /* NMEA output cycle: 2 seconds */
    if (at_ok_worker("AT+MGNSSCFG=\"nmea/cycle\",2\r\n", AT_TIMEOUT_MS) < 0)
        fprintf(stderr, "[ML307C] set nmea/cycle failed\n");

    /* Configure AGNSS server URL */
    if (at_ok_worker("AT+MGNSSCFG=\"agnss/url\",\"cmiot-api1.rx-networks.cn:80\"\r\n", AT_TIMEOUT_MS) < 0)
        fprintf(stderr, "[ML307C] set AGNSS URL failed (non-fatal)\n");

    /* Enable auto-report location info */
    if (at_ok_worker("AT+MGNSSLOC=1\r\n", AT_TIMEOUT_MS) < 0)
        fprintf(stderr, "[ML307C] set MGNSSLOC failed\n");

    /* Request AGNSS data update (async - OK means accepted, URC comes later) */
    printf("[ML307C] requesting AGNSS data update...\n");
    if (at_ok_worker("AT+MAGNSSDATA\r\n", AT_TIMEOUT_MS) < 0) {
        fprintf(stderr, "[ML307C] AGNSS data request failed (non-fatal)\n");
    } else {
        /* Wait for URC: +MAGNSSDATA: 1 */
        g_agnss_data_ready = 0;  /* Reset flag */
        
        if (wait_for_urc(&g_agnss_data_ready, "+MAGNSSDATA: 1", 5000) == 0)
            printf("[ML307C] AGNSS data updated successfully\n");
        else
            fprintf(stderr, "[ML307C] AGNSS data download timeout (non-fatal)\n");
    }

    /* Enable AGNSS mode: check if already enabled first */
    printf("[ML307C] checking AGNSS status...\n");
    int n = at_send_worker("AT+MAGNSSEN?\r\n", AT_TIMEOUT_MS);
    int agnss_enabled = 0;
    
    if (n > 0 && strstr(g_at_resp, "+MAGNSSEN: 1")) {
        agnss_enabled = 1;
        printf("[ML307C] AGNSS already enabled, skipping\n");
    } else {
        printf("[ML307C] enabling AGNSS...\n");
        if (at_ok_worker("AT+MAGNSSEN=1\r\n", AT_TIMEOUT_MS) < 0) {
            fprintf(stderr, "[ML307C] enable AGNSS failed (non-fatal)\n");
        } else {
            printf("[ML307C] AGNSS enabled\n");
        }
    }
    
    (void)agnss_enabled;

    /* Start GNSS: check if already running first */
    printf("[ML307C] checking GNSS status...\n");
    n = at_send_worker("AT+MGNSS?\r\n", AT_TIMEOUT_MS);
    int gnss_running = 0;
    
    if (n > 0 && strstr(g_at_resp, "+MGNSS: 1")) {
        gnss_running = 1;
        printf("[ML307C] GNSS already running, skipping start\n");
    } else {
        printf("[ML307C] starting GNSS...\n");
        if (at_ok_worker("AT+MGNSS=1\r\n", AT_TIMEOUT_MS) < 0) {
            fprintf(stderr, "[ML307C] start GNSS failed\n");
            return -1;
        }
        printf("[ML307C] GNSS started\n");
    }
    
    (void)gnss_running;  /* Used for status logging */

    printf("[ML307C] GNSS configured: AGNSS on, GNRMC only, cycle=2s\n");
    return 0;
}

/* ======================== Worker Thread ======================== */

/*
 * Initialize ML307C module (called from worker thread).
 * Returns 0 on success, -1 on failure.
 */
static int ml307c_do_init(void)
{
    printf("[ML307C] initializing module...\n");

    /* Wait for module ready */
    if (wait_module_ready() < 0) {
        fprintf(stderr, "[ML307C] module init failed\n");
        return -1;
    }

    /* Disable echo */
    if (at_ok_worker("ATE0\r\n", AT_TIMEOUT_MS) < 0)
        fprintf(stderr, "[ML307C] ATE0 failed (non-fatal)\n");

    /* Wait for network */
    if (wait_network(60) < 0)
        fprintf(stderr, "[ML307C] no network, continuing anyway\n");

    /* RNDIS dial-up */
    if (rndis_dial() < 0)
        fprintf(stderr, "[ML307C] RNDIS dial failed\n");

    /* Configure GNSS */
    if (gnss_configure() < 0) {
        fprintf(stderr, "[ML307C] GNSS config failed\n");
        return -1;
    }

    printf("[ML307C] module initialized successfully\n");
    return 0;
}

/*
 * Worker thread: handles ALL ML307C operations.
 * - Monitors USB device availability
 * - Opens port and initializes when device appears
 * - Re-initializes on disconnect/reconnect
 * - Main loop: reads AT port, processes NMEA/AT responses
 */
static void *worker_thread(void *arg)
{
    (void)arg;
    char buf[1024];
    g_line_pos = 0;
    int initialized = 0;  /* Track if module is initialized */
    int64_t last_signal_ms = 0;  /* Track last signal strength query */

    printf("[ML307C] worker thread started\n");

    while (g_worker_running) {
        /* ---- Phase 1: Wait for device to appear ---- */
        if (access(ML307C_AT_PORT, F_OK) != 0) {
            if (initialized) {
                fprintf(stderr, "[ML307C] device disconnected\n");
                initialized = 0;
            }
            
            printf("[ML307C] waiting for %s...\n", ML307C_AT_PORT);
            /* Poll every 100ms for up to 10s */
            for (int i = 0; i < 100 && g_worker_running; i++) {
                if (access(ML307C_AT_PORT, F_OK) == 0)
                    break;
                ml307c_sleep(100);
            }
            continue;
        }

        /* ---- Phase 2: Device exists, open port ---- */
        if (g_at_fd < 0) {
            g_at_fd = serial_open(ML307C_AT_PORT);
            if (g_at_fd < 0) {
                fprintf(stderr, "[ML307C] failed to open port, retrying...\n");
                ml307c_sleep(2000);
                continue;
            }
            printf("[ML307C] AT port %s opened\n", ML307C_AT_PORT);
        }

        /* ---- Phase 3: Initialize module (if not already) ---- */
        if (!initialized) {
            if (ml307c_do_init() == 0) {
                initialized = 1;
            } else {
                fprintf(stderr, "[ML307C] init failed, will retry...\n");
                /* Close and retry */
                if (g_at_fd >= 0) {
                    close(g_at_fd);
                    g_at_fd = -1;
                }
                ml307c_sleep(5000);  /* Wait 5s before retry */
                continue;
            }
        }

        /* ---- Phase 4: Main loop - read and process data ---- */
        
        /* Query signal strength every 5 seconds */
        int64_t now = now_ms();
        if (initialized && now - last_signal_ms >= 5000) {
            int n = at_send_worker("AT+CSQ\r\n", AT_TIMEOUT_MS);
            if (n > 0) {
                /* Response format: +CSQ: <rssi>,<ber> */
                char *csq = strstr(g_at_resp, "+CSQ:");
                if (csq) {
                    int rssi = 0;
                    if (sscanf(csq, "+CSQ: %d", &rssi) == 1) {
                        /* Convert RSSI to dBm
                         * RSSI 0: <= -113 dBm
                         * RSSI 1: -111 dBm
                         * RSSI 2-31: -109 to -51 dBm (2 dBm steps)
                         * RSSI 99: unknown
                         */
                        int16_t dbm = 0;
                        if (rssi == 0) {
                            dbm = -113;
                        } else if (rssi >= 1 && rssi <= 31) {
                            dbm = -111 + (rssi - 1) * 2;
                        } else {
                            dbm = 0; /* Unknown */
                        }
                        
                        pthread_mutex_lock(&g_signal_mtx);
                        g_signal_dbm = dbm;
                        g_signal_update_ms = now;
                        pthread_mutex_unlock(&g_signal_mtx);
                        
                        // printf("[ML307C] Signal: RSSI=%d, %d dBm\n", rssi, dbm);
                    }
                }
            }
            last_signal_ms = now;
        }
        
        ssize_t n = read(g_at_fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* No data, check for pending commands */
                pthread_mutex_lock(&g_at_cmd_mtx);
                if (g_at_cmd_has_pending) {
                    char cmd[256];
                    strcpy(cmd, g_at_cmd_pending);
                    g_at_cmd_has_pending = 0;
                    pthread_mutex_unlock(&g_at_cmd_mtx);
                    at_send_worker(cmd, AT_TIMEOUT_MS);
                    continue;
                }
                pthread_mutex_unlock(&g_at_cmd_mtx);
                usleep(10000);  /* 10ms */
                continue;
            }
            if (errno == EINTR) continue;
            
            /* Read error - device likely disconnected */
            fprintf(stderr, "[ML307C] read error: %s, closing port\n", strerror(errno));
            if (g_at_fd >= 0) {
                close(g_at_fd);
                g_at_fd = -1;
            }
            initialized = 0;
            ml307c_sleep(2000);  /* Wait before retry */
            continue;
        }
        if (n == 0) {
            usleep(10000);
            continue;
        }

        /* Process data line by line */
        for (int i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\r' || c == '\n') {
                if (g_line_pos > 0) {
                    g_line_buf[g_line_pos] = '\0';
                    process_line(g_line_buf);
                    g_line_pos = 0;
                }
            } else if (g_line_pos < (int)sizeof(g_line_buf) - 1) {
                g_line_buf[g_line_pos++] = c;
            }
        }
    }

    printf("[ML307C] worker thread shutting down\n");
    
    /* Cleanup */
    if (g_at_fd >= 0) {
        close(g_at_fd);
        g_at_fd = -1;
    }
    
    printf("[ML307C] worker thread exited\n");
    return NULL;
}

/* ======================== Public API ======================== */

int ml307c_init(void)
{
    memset(&g_gps, 0, sizeof(g_gps));
    g_line_pos = 0;
    g_at_resp_len = 0;
    g_at_resp_ready = 0;
    g_at_cmd_has_pending = 0;

    /* Start worker thread - it will handle everything */
    g_worker_running = 1;
    if (pthread_create(&g_worker_tid, NULL, worker_thread, NULL) != 0) {
        fprintf(stderr, "[ML307C] create worker thread failed\n");
        return -1;
    }

    return 0;
}

void ml307c_deinit(void)
{
    if (!g_worker_running)
        return;

    g_worker_running = 0;

    /* Wakeup any blocked read by closing fd */
    if (g_at_fd >= 0) {
        close(g_at_fd);
        g_at_fd = -1;
    }

    pthread_join(g_worker_tid, NULL);
    printf("[ML307C] deinitialized\n");
}

int ml307c_get_gps(ml307c_gps_data_t *data)
{
    if (!data) return 0;

    pthread_mutex_lock(&g_gps_mtx);
    if (g_gps.valid && g_gps.update_ms > 0) {
        *data = g_gps;
        pthread_mutex_unlock(&g_gps_mtx);
        return 1;
    }
    pthread_mutex_unlock(&g_gps_mtx);
    return 0;
}

/*
 * Get the latest signal strength in dBm.
 * Returns signal strength (negative value, e.g. -75), or 0 if unknown.
 */
int16_t ml307c_get_signal(void)
{
    int16_t signal = 0;
    
    pthread_mutex_lock(&g_signal_mtx);
    signal = g_signal_dbm;
    pthread_mutex_unlock(&g_signal_mtx);
    
    return signal;
}
