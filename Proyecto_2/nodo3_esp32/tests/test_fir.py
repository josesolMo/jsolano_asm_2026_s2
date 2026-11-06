"""Prueba en el PC de 'mode fir' del Nodo 3 (salida_fir.h): FIR del Nodo 2 alineado con HOP.

Comprueba, para los 4 perfiles:
  1. y[t] = (h * x)[t - (HOP - D)]: es exactamente el FIR del Nodo 2 (scipy.lfilter con los
     mismos coeficientes), con el retardo total de HOP muestras que tienen 'raw' y 'filter'.
  2. Un tono dentro de la banda de paso sale alineado con x[t - HOP] (fase lineal).
  3. La atenuacion que mide StatsSenales en la banda de rechazo coincide con |H(f)|.
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import numpy as np
from scipy import signal as sg

FW = Path(__file__).resolve().parents[1]
FS, HOP = 16000, 512
# perfil: (paso de prueba Hz, rechazo de prueba Hz)
PERFILES = {0: ("TONO", 1000, 2500), 1: ("MULTITONO", 880, 3000), 2: ("AUDIO", 1500, 5000), 3: ("TRANSITORIA", 600, 2500)}


def coeficientes(nombre):
    s = (FW / "coeficientes.h").read_text()
    m = re.search(rf"{nombre} = \{{\{{(.*?)\}}\}};", s, re.S)
    return np.array([float(v.rstrip("f")) for v in re.findall(r"-?\d+\.\d+(?:e-?\d+)?f?", m.group(1))])


def main():
    tmp = Path(tempfile.mkdtemp())
    exe = tmp / "host_fir"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(exe),
                    str(FW / "tests/host_fir.cpp"), str(FW / "fft_processing.cpp")], check=True)
    rng = np.random.default_rng(1)
    t = np.arange(64 * HOP) / FS
    fallas = 0
    for perfil, (nombre, f_paso, f_rech) in PERFILES.items():
        h = coeficientes(nombre)
        D = (len(h) - 1) // 2
        # 1. ruido blanco: igualdad muestra a muestra con lfilter + retardo HOP - D
        x = (0.3 * rng.standard_normal(t.size)).astype(np.float32)
        x.tofile(tmp / "x.bin")
        out = subprocess.run([str(exe), str(perfil), str(tmp / "x.bin"), str(tmp / "y.bin"), "0", "1"],
                             check=True, capture_output=True, text=True).stdout
        y = np.fromfile(tmp / "y.bin", dtype=np.float32)
        ref = np.concatenate([np.zeros(HOP - D), sg.lfilter(h, 1, x.astype(np.float64))])[:x.size]
        err = np.max(np.abs(y - ref))
        ok1 = err < 1e-5 and f"retardo_fir={D}" in out
        # 2. tono en la banda de paso: y[t] ~ |H| x[t - HOP]
        x = (0.5 * np.sin(2 * np.pi * f_paso * t)).astype(np.float32)
        x.tofile(tmp / "x.bin")
        subprocess.run([str(exe), str(perfil), str(tmp / "x.bin"), str(tmp / "y.bin"), "0", "1"], check=True,
                       capture_output=True)
        y = np.fromfile(tmp / "y.bin", dtype=np.float32)
        g = abs(sg.freqz(h, worN=[f_paso], fs=FS)[1][0])
        res = y[4 * HOP:] - g * x[3 * HOP:-HOP]
        ok2 = np.max(np.abs(res)) < 2e-3
        # 3. tono en la banda de rechazo: atenuacion medida = -20 log10 |H|
        x = (0.5 * np.sin(2 * np.pi * f_rech * t) + 0.3 * np.sin(2 * np.pi * f_paso * t)).astype(np.float32)
        x.tofile(tmp / "x.bin")
        out = subprocess.run([str(exe), str(perfil), str(tmp / "x.bin"), str(tmp / "y.bin"),
                              str(f_rech - 50), str(f_rech + 50)], check=True, capture_output=True, text=True).stdout
        aten = float(re.search(r"atenuacion_rango_db=([-\d.]+)", out).group(1))
        teorica = -20 * np.log10(abs(sg.freqz(h, worN=[f_rech], fs=FS)[1][0]))
        ok3 = abs(aten - teorica) < 1.0
        bien = ok1 and ok2 and ok3
        fallas += not bien
        print(f"{nombre:12s} D={D:2d} | igual a lfilter (err {err:.1e}) {'OK' if ok1 else 'FALLA'} | "
              f"alineado con raw {'OK' if ok2 else 'FALLA'} | atenuacion en {f_rech} Hz: medida {aten:.1f} dB, "
              f"teorica {teorica:.1f} dB {'OK' if ok3 else 'FALLA'}")
    print("RESULTADO FIR:", "OK" if not fallas else f"{fallas} perfil(es) con fallas")
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
