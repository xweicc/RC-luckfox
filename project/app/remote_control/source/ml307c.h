/*
 * ml307c.h - ML307C 4G module driver interface
 *
 * Handles AT command communication, RNDIS dial-up, and GNSS positioning
 * via the ML307C module on /dev/ttyUSB2 (AT port).
 *
 * Note: ML307C does not support nmea/port=1 (USB GPS port output).
 * NMEA data is output on the AT port (nmea/port=0) as unsolicited
 * result codes (URC) and parsed in the worker thread.
 *
 * Features:
 * - Automatic USB hot-plug support (disconnect/reconnect)
 * - Waits for device if not present at startup
 * - Auto-reinitializes on reconnect
 */

#ifndef ML307C_H
#define ML307C_H

#include <stdint.h>

#define ML307C_AT_PORT  "/dev/ttyUSB2"

/* Parsed GPS data from GNRMC */
typedef struct {
    double   latitude;    /* Decimal degrees, +N / -S */
    double   longitude;   /* Decimal degrees, +E / -W */
    float    speed_kmh;   /* Speed in km/h */
    int      valid;       /* 1 = fix valid (A), 0 = no fix (V) */
    int64_t  update_ms;   /* Timestamp of last update */
} ml307c_gps_data_t;

/*
 * Initialize the ML307C module:
 *   Starts a worker thread that handles ALL operations:
 *   1. Monitors USB device availability
 *   2. Opens AT serial port when device appears
 *   3. Initializes module (AT commands, RNDIS, GNSS)
 *   4. Enters main loop to process NMEA/AT responses
 *   5. Handles USB disconnect/reconnect automatically
 *
 * Returns 0 on success (thread started), -1 on failure.
 * Note: This returns immediately; initialization happens in background thread.
 */
int ml307c_init(void);

/*
 * Cleanup: stop worker thread, close serial port.
 */
void ml307c_deinit(void);

/*
 * Get the latest GPS data. Thread-safe.
 * Returns 1 if data was copied (valid fix available), 0 otherwise.
 */
int ml307c_get_gps(ml307c_gps_data_t *data);

/*
 * Get the IMEI of the modem module.
 * Returns 1 if IMEI was copied to buf, 0 if not yet available.
 * IMEI is fetched during initialization via AT+GSN=1.
 */
int ml307c_get_imei(char *buf, int size);

/*
 * Get the latest signal strength in dBm.
 * Returns signal strength (negative value, e.g. -75), or 0 if unknown.
 * Updated every 5 seconds by the worker thread.
 */
int16_t ml307c_get_signal(void);

#endif /* ML307C_H */
