#!/usr/bin/env bash
# ==============================================================================
# T-1.2 & T-1.3: Launch Micro XRCE-DDS Agent for Pixhawk TELEM2 (Baudrate 921600)
# ==============================================================================

PORT="${1:-/dev/ttyAMA0}"
BAUDRATE="${2:-921600}"

echo "[+] Launching MicroXRCEAgent on port $PORT with baudrate $BAUDRATE..."
MicroXRCEAgent serial --dev "$PORT" -b "$BAUDRATE"
