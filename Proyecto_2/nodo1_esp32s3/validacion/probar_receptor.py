"""Compila receptor.h en la PC y lo compara bloque a bloque con receptor_fft.py.

No necesita la placa. Ejecutar desde la carpeta que contiene nodo1_esp32s3/:
    python3 nodo1_esp32s3/validacion/probar_receptor.py
Requiere g++ (o clang++), numpy y scipy.

Se comprueban dos cosas:
  1. Paso 2: el espectro |X[k]|^2 de la FFT en C++ (float32) coincide con numpy.
  2. Paso 3: las frecuencias validadas por C++ coinciden con las de Python.
"""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np

CARPETA = Path(__file__).resolve().parent
NODO = CARPETA.parent
sys.path.insert(0, str(NODO.parent))
from receptor_fft.src.receptor_fft import (  # noqa: E402
    FS, N_BLOQUE, SALTO, Receptor, espectro_potencia)

# Programa de escritorio: lee muestras float64, imprime el espectro del primer
# bloque y luego las frecuencias validadas de cada bloque (salto de N/2).
PROGRAMA = r'''
#include "../receptor.h"
#include <cstdio>
#include <vector>
int main(int argc, char** argv) {
    FILE* f = std::fopen(argv[1], "rb");
    std::vector<double> x;
    double v;
    while (std::fread(&v, sizeof v, 1, f) == 1) x.push_back(v);
    std::fclose(f);
    static nodo1::Receptor rx;
    std::vector<float> bloque(nodo1::N);
    for (size_t inicio = 0; inicio + nodo1::N <= x.size(); inicio += nodo1::SALTO) {
        for (int n = 0; n < nodo1::N; ++n) bloque[n] = static_cast<float>(x[inicio + n]);
        rx.analizar(bloque.data());
        if (inicio == 0) {
            std::printf("P");
            for (int k = 0; k < nodo1::BINS; ++k) std::printf(" %.9g", rx.potencia[k]);
            std::printf("\n");
        }
        std::printf("B");
        for (int i = 0; i < rx.n_validadas; ++i) {
            const auto& s = rx.validadas[i];
            std::printf(" %.6f %.6g %.4f %d", s.f_hz, s.amplitud, s.snr_db, s.flags);
        }
        std::printf("\n");
    }
    return 0;
}
'''


def comprobar(condicion, mensaje):
    # Una excepcion explicita mantiene la comprobacion aun con python -O.
    if not condicion:
        raise AssertionError(mensaje)


def compilar(tmp):
    compilador = shutil.which("g++") or shutil.which("clang++")
    comprobar(compilador, "No se encontro g++ ni clang++.")
    fuente = CARPETA / "_prueba_receptor.cpp"
    fuente.write_text(PROGRAMA)
    try:
        exe = tmp / "receptor"
        subprocess.run([compilador, "-std=c++17", "-O2", str(fuente), "-o", str(exe)], check=True)
    finally:
        fuente.unlink()
    return exe


def correr_cpp(exe, x, tmp):
    entrada = tmp / "x.bin"
    np.asarray(x, dtype="<f8").tofile(entrada)
    lineas = subprocess.run([str(exe), str(entrada)], check=True, capture_output=True,
                            text=True).stdout.splitlines()
    potencia = np.array(lineas[0].split()[1:], dtype=float)
    bloques = []
    for linea in lineas[1:]:
        v = linea.split()[1:]
        bloques.append([(float(v[i]), float(v[i + 1]), float(v[i + 2]), int(v[i + 3]))
                        for i in range(0, len(v), 4)])
    return potencia, bloques


def correr_python(x):
    rx = Receptor()
    return [[(s['f_hz'], s['amplitud'], s['snr_local_db'], s['flags']) for s in rx.procesar(x[i:i + N_BLOQUE])]
            for i in range(0, len(x) - N_BLOQUE + 1, SALTO)]


def senales():
    """Casos de prueba: los modos del emisor por un ADC de 12 bits y otros del taller."""
    rng = np.random.default_rng(2026)
    t = np.arange(4 * FS) / FS
    adc = lambda v: np.clip(np.round(v * 4095 + 3 * rng.standard_normal(len(v))), 0, 4095) / 4095
    centro = 0.5
    tres = (np.sin(2 * np.pi * 440 * t) + .55 * np.sin(2 * np.pi * 880 * t + .61)
            + .30 * np.sin(2 * np.pi * 1320 * t - .87))
    red = sum(a * np.sin(2 * np.pi * 60 * h * t) for h, a in ((1, .2), (3, .08), (5, .05)))
    yield "1000 Hz", adc(centro + 0.25 * np.sin(2 * np.pi * 1000 * t))
    yield "2500 Hz", adc(centro + 0.25 * np.sin(2 * np.pi * 2500 * t))
    yield "440/880/1320 Hz", adc(centro + 0.15 * tres)
    yield "tres tonos + 2513.7 Hz", adc(centro + 0.1 * tres + 0.08 * np.sin(2 * np.pi * 2513.7 * t))
    yield "red 60 Hz + armonicos", adc(centro + 0.12 * tres + 0.5 * red)
    yield "solo ruido", adc(centro + 0.05 * rng.standard_normal(len(t)))
    yield "salto 2500 -> 3300 Hz", adc(centro + 0.1 * tres + 0.08 * np.where(
        t < 2, np.sin(2 * np.pi * 2500 * t), np.sin(2 * np.pi * 3300 * t)))


def comparar(nombre, py, cpp):
    """Cuenta bloques identicos: mismas frecuencias (±0.05 Hz) y mismas banderas."""
    iguales = 0
    df_max = 0.0
    for a, b in zip(py, cpp):
        if len(a) == len(b) and all(sa[3] == sb[3] and abs(sa[0] - sb[0]) < 0.05 for sa, sb in zip(a, b)):
            iguales += 1
            for sa, sb in zip(a, b):
                df_max = max(df_max, abs(sa[0] - sb[0]))
    total = len(py)
    print(f"  {nombre:24s} {iguales:3d}/{total} bloques iguales, |df| max {df_max:.4f} Hz")
    return iguales, total


def main():
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        exe = compilar(tmp)
        print("Paso 2 (FFT float32 frente a numpy) y paso 3 (receptor C++ frente a Python):")
        total_iguales = total_bloques = 0
        peor_fft = 0.0
        for nombre, x in senales():
            potencia, cpp = correr_cpp(exe, x, tmp)
            ref = espectro_potencia(x[:N_BLOQUE])
            # Error relativo al maximo: los bins pequenos no pesan mas que los grandes.
            peor_fft = max(peor_fft, float(np.max(np.abs(potencia - ref)) / np.max(ref)))
            iguales, total = comparar(nombre, correr_python(x), cpp)
            total_iguales += iguales
            total_bloques += total
        print(f"FFT: error maximo relativo {peor_fft:.2e}")
        print(f"Receptor: {total_iguales}/{total_bloques} bloques identicos")
        comprobar(peor_fft < 1e-4, "La FFT en C++ se aleja de numpy")
        comprobar(total_iguales >= 0.98 * total_bloques, "El receptor en C++ difiere de Python")
        print("OK")


if __name__ == "__main__":
    main()