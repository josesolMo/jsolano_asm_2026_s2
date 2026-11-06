#!/usr/bin/env bash
# Todas las pruebas de PC del proyecto (no usan placas). Requiere g++, python3, numpy y scipy.
# Uso, desde Proyecto_2/:  bash probar_todo.sh      (PY=ruta/a/python para otro interprete)
set -eu
PY="${PY:-python3}"
cd "$(dirname "$0")"
echo "== Nodo 1: tramas (trama.h contra link.py)";      "$PY" nodo1_esp32s3/validacion/probar_trama.py
echo "== Nodo 1: receptor (receptor.h contra Python)";  "$PY" nodo1_esp32s3/validacion/probar_receptor.py
echo "== Enlace nodo 1 -> nodo 3 (UART)";               "$PY" nodo1_esp32s3/validacion/probar_enlace.py
echo "== Nodo 3: parser, DSP y modo fir";              bash nodo3_esp32/ejecutar_pruebas.sh
echo "== Nodo 2: FIR en C++ contra el diseno"
# verificar.py reescribe resultados.json; se restaura para no ensuciar git
R=nodo2_temporal/validacion/resultados.json; cp "$R" "$R.bak"
(cd .. && "$PY" Proyecto_2/nodo2_temporal/validacion/verificar.py > /dev/null) && mv "$R.bak" "$R" && echo "OK"
echo "TODAS LAS PRUEBAS DE PC TERMINARON SIN ERRORES"
