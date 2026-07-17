#!/bin/bash
#
# rproxys installation script for Ubuntu 24.04 (systemd)
#

set -e

PKG_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DST="/usr/bin/rproxys"
CONF_DST="/etc/rproxys.conf"
SVC_DST="/etc/systemd/system/rproxys.service"

echo "========================================"
echo "  rproxys installer (systemd)"
echo "========================================"
echo ""

# Check root
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root (use sudo)."
    exit 1
fi

# 1. Install binary
echo "[1/3] Installing binary to ${BIN_DST} ..."
cp -f "${PKG_DIR}/rproxys" "${BIN_DST}"
chmod +x "${BIN_DST}"
echo "      Done."

# 2. Install config (preserve existing)
if [ -f "${CONF_DST}" ]; then
    echo "[2/3] Config ${CONF_DST} already exists, keeping current config."
else
    echo "[2/3] Installing default config to ${CONF_DST} ..."
    cp -f "${PKG_DIR}/rproxys.conf" "${CONF_DST}"
    echo "      Done."
fi

# 3. Install systemd service
echo "[3/3] Installing systemd service ..."
cp -f "${PKG_DIR}/rproxys.service" "${SVC_DST}"
systemctl daemon-reload
systemctl enable rproxys.service
systemctl restart rproxys.service
echo "      Done."

echo ""
echo "========================================"
echo "  Installation complete!"
echo "  Binary : ${BIN_DST}"
echo "  Config : ${CONF_DST}"
echo "  Service: ${SVC_DST}"
echo "========================================"
echo ""
echo "rproxys is running and will auto-start on boot."
echo "Manage with: systemctl {start|stop|restart|status} rproxys"
