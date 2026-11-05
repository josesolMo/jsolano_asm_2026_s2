"""Compila trama.h en la PC y comprueba que sus tramas las lee link.py sin errores.

No necesita la placa. Ejecutar desde taller-fft-sys/:
    python nodo1_esp32s3/validacion/probar_trama.py
Requiere un compilador de C++ (g++ o clang++) en el PATH.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np

CARPETA = Path(__file__).resolve().parent
TALLER = CARPETA.parents[1]
# config.py de full_system crea "taller-fft-sys/resultados_sistema" relativo a la
# carpeta actual, asi que se importa link.py parado en la raiz del repositorio.
os.chdir(TALLER.parent)
sys.path.insert(0, str(TALLER / "full_system"))
from sistema_ruido.link import crc16, desempaquetar_flujo, empaquetar  # noqa: E402

# Programa de escritorio: arma tres tramas con muestras conocidas y las escribe en un archivo.
PRUEBA = r'''
#include "../trama.h"
#include <cstdio>
#include <vector>
int main(int argc, char** argv) {
    const size_t n = 1024;
    std::vector<int16_t> muestras(n);
    std::vector<uint8_t> salida(trama::tamano(n));
    FILE* f = std::fopen(argv[1], "wb");
    for (uint16_t seq = 0; seq < 3; ++seq) {
        // Rampa de 0 a 4095 desplazada por la secuencia: cubre todo el rango del ADC.
        for (size_t i = 0; i < n; ++i) muestras[i] = static_cast<int16_t>((i * 4 + seq) % 4096);
        size_t largo = trama::empaquetar(seq, muestras.data(), n, salida.data());
        std::fwrite(salida.data(), 1, largo, f);
    }
    std::fclose(f);
    return 0;
}
'''


def comprobar(condicion, mensaje):
    # Una excepcion explicita mantiene la comprobacion aun con python -O.
    if not condicion:
        raise AssertionError(mensaje)


def main():
    compilador = shutil.which("g++") or shutil.which("clang++")
    comprobar(compilador, "No se encontro g++ ni clang++.")
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        fuente = CARPETA / "_prueba_trama.cpp"
        fuente.write_text(PRUEBA)
        try:
            exe = tmp / "prueba"
            subprocess.run([compilador, "-std=c++17", "-O2", str(fuente), "-o", str(exe)], check=True)
            salida = tmp / "tramas.bin"
            subprocess.run([str(exe), str(salida)], check=True)
            flujo = salida.read_bytes()
        finally:
            fuente.unlink()

    # 1) link.py debe aceptar las tres tramas tal cual.
    validas = desempaquetar_flujo(flujo)
    comprobar(sorted(validas) == [0, 1, 2], f"Secuencias leidas: {sorted(validas)}")
    for seq, muestras in validas.items():
        esperadas = (np.arange(1024) * 4 + seq) % 4096
        comprobar(np.array_equal(muestras, esperadas), f"Muestras distintas en la trama {seq}")

    # 2) Los bytes deben ser identicos a los que arma link.py en Python.
    referencia = b"".join(empaquetar(s, ((np.arange(1024) * 4 + s) % 4096).astype(np.int16))
                          for s in range(3))
    comprobar(flujo == referencia, "Los bytes de trama.h difieren de link.py")

    # 3) Un bit cambiado debe hacer que link.py descarte la trama por CRC.
    danado = bytearray(flujo)
    danado[100] ^= 0x01
    comprobar(sorted(desempaquetar_flujo(bytes(danado))) == [1, 2], "Un error no fue detectado")

    print(f"OK: 3 tramas identicas a link.py, CRC verificado (crc de prueba {crc16(b'123456789'):#06x}).")


if __name__ == "__main__":
    main()
