#!/usr/bin/env bash
# Compila (y opcionalmente carga) el Nodo 3 con arduino-cli. Uso:
#   ./compilar.sh              solo compila
#   ./compilar.sh /dev/ttyUSB0 compila y carga en ESE puerto (verifiquelo antes con: arduino-cli board list)
set -euo pipefail
CLI="${ARDUINO_CLI:-arduino-cli}"
FQBN="esp32:esp32:esp32"                    # ESP32 Dev Module (ESP-WROOM-32)
CORE_VER="3.3.12"                           # la misma version que el emisor
URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
AQUI="$(cd "$(dirname "$0")" && pwd)"

"$CLI" config init >/dev/null 2>&1 || true
"$CLI" config add board_manager.additional_urls "$URL" >/dev/null 2>&1 || true
"$CLI" core update-index
if ! "$CLI" core list | grep -q "esp32:esp32 *$CORE_VER"; then
  "$CLI" core install "esp32:esp32@$CORE_VER"
fi
"$CLI" compile --fqbn "$FQBN" --warnings all --build-path "${BUILD_DIR:-/tmp/nodo3_build}" "$AQUI"
if [ "${1:-}" != "" ]; then
  echo "Cargando en $1 ..."
  "$CLI" upload -p "$1" --fqbn "$FQBN" --input-dir "${BUILD_DIR:-/tmp/nodo3_build}" "$AQUI"
fi
