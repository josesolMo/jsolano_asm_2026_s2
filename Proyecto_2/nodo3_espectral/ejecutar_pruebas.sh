#!/usr/bin/env bash
# Pruebas de software en el PC (NO prueban la placa). Requiere g++, python3, numpy, scipy.
#   python3 -m venv .venv && .venv/bin/pip install numpy scipy matplotlib pyserial
#   PY=.venv/bin/python ./ejecutar_pruebas.sh
set -euo pipefail
PY="${PY:-python3}"
RAIZ="$(cd "$(dirname "$0")/.." && pwd)"          # Proyecto_2/
cd "$RAIZ"
"$PY" nodo3_espectral/tests/test_link.py
"$PY" nodo3_espectral/tests/test_dsp_vs_python.py
