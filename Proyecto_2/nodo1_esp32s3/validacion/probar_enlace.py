"""Prueba sin placas del enlace nodo 1 -> nodo 3 (UART).

Ejecutar desde la carpeta que contiene nodo1_esp32s3/ y nodo3_esp32/:
    python3 nodo1_esp32s3/validacion/probar_enlace.py
Requiere g++, numpy y scipy.

1. Genera los escenarios P1..P5 de full_system cuantizados como el ADC (0..4095).
2. enlace_host.cpp corre el codigo del nodo 1 (receptor.h, trama.h, piso_nodo3.h)
   y produce el flujo de bytes que saldria por la UART1.
3. Ese flujo entra al parser REAL del nodo 3 (communication.cpp, por host_link.cpp)
   en trozos de tamano variable. Se comprueba:
   - todas las tramas llegan, sin errores de CRC ni bytes basura;
   - las muestras PCM son las mismas que se enviaron;
   - las frecuencias que entiende el nodo 3 son las que valido el nodo 1;
   - el piso coincide con el estimador de Python (acquisition.py), que es el
     mismo que el nodo 3 calcularia por su cuenta (diag local).
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

import numpy as np

CARPETA = Path(__file__).resolve().parent
NODO1 = CARPETA.parent
RAIZ = NODO1.parent                                   # Proyecto_2/
NODO3 = RAIZ / "nodo3_esp32"
sys.path.insert(0, str(RAIZ / "full_system"))
os.chdir(RAIZ.parent)                                 # config.py crea carpetas relativas a esta
from sistema_ruido import scenarios, signals          # noqa: E402
from sistema_ruido.config import FS, L, N, PICO, ADC_BITS   # noqa: E402
from sistema_ruido.acquisition import NodoAdquisicion       # noqa: E402

TOL_CENTI_DB = 5                                      # 0.05 dB (float32 frente a float64)


def senal_adc(id_esc):
    esc = scenarios.escenario(id_esc)
    rng = np.random.default_rng(esc["semilla"])
    t = np.arange(L) / FS
    limpia = signals.fade(signals.generar_senal(t=t, **esc["senal"]))
    ruido = signals.escalar_para_snr(limpia, signals.generar_ruido(n=L, rng=rng, **esc["ruido"]), esc["snr_db"])
    g = PICO / max(np.abs(limpia).max(), np.abs(limpia + ruido).max())
    adc_f = signals.cuantizar(g * (limpia + ruido), ADC_BITS)
    return (np.round(adc_f * 2048) + 2048).astype("<i2"), esc


def fnv(bloque):
    h = 2166136261
    for v in bloque.astype(np.uint16):
        h = ((h ^ int(v)) * 16777619) & 0xFFFFFFFF
    return h


def main():
    tmp = Path(tempfile.mkdtemp())
    n1, n3 = tmp / "enlace_host", tmp / "host_link"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-o", str(n1),
                    str(CARPETA / "enlace_host.cpp")], check=True)
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(n3),
                    str(NODO3 / "tests/host_link.cpp"), str(NODO3 / "communication.cpp")], check=True)

    fallas = 0
    for id_esc in ["P1", "P2", "P3", "P4", "P5"]:
        adc, esc = senal_adc(id_esc)
        adc.tofile(tmp / "codigos.bin")
        enviado = subprocess.run([str(n1), str(tmp / "codigos.bin"), str(tmp / "flujo.bin")],
                                 check=True, capture_output=True, text=True).stdout.split("\n")
        enviado = [l.split() for l in enviado if l]
        # trozo de 97 bytes: las tramas llegan partidas en lugares arbitrarios
        lineas = subprocess.run([str(n3), str(tmp / "flujo.bin"), "97"], check=True,
                                capture_output=True, text=True).stdout.split("\n")
        pcm = [l.split() for l in lineas if l.startswith("PCM")]
        par = [l.split() for l in lineas if l.startswith("PARAMS")]
        pis = [l.split() for l in lineas if l.startswith("PISO")]
        stats = [l for l in lineas if l and not l.startswith(("PCM", "PARAMS", "PISO"))]
        nb = len(adc) // N

        # PCM: todas, en orden y con las mismas muestras
        ok_pcm = len(pcm) == nb and all(int(p[1]) == i and int(p[2], 16) == fnv(adc[i * N:(i + 1) * N])
                                         for i, p in enumerate(pcm))
        # PARAMS: lo que entiende el nodo 3 es lo que valido el nodo 1 (f con resolucion 0.25 Hz)
        ok_par = len(par) == nb == len(enviado)
        for p, e in zip(par, enviado):
            if p[1] != e[1] or p[2] != e[2]:
                ok_par = False
                continue
            for a, b in zip(p[3:], e[3:]):
                fa, _, fla = a.split(",")
                fb, flb = b.split(",")
                ok_par &= abs(float(fa) - float(fb)) <= 0.125 + 1e-6 and fla == flb
        # PISO: igual al de acquisition.py (lo que el nodo 3 calcularia en 'diag local')
        ref = NodoAdquisicion(esc["banda_util"])
        dif_max = 0
        for i, p in enumerate(pis):
            x = (adc[i * N:(i + 1) * N].astype(np.float64) - 2048) / 2048
            piso = ref.procesar(x).piso
            esperado = np.round(1000 * np.log10(np.maximum(piso[::8], 1e-30))).astype(int)
            recibido = np.array([int(v) for v in p[4:]])
            assert int(p[2]) == 8 and int(p[3]) == len(esperado) == 65
            dif_max = max(dif_max, int(np.abs(recibido - esperado).max()))
        ok_pis = len(pis) == nb and dif_max <= TOL_CENTI_DB
        ok_stats = any("crc_err=0" in s for s in stats) and any("descartados=0" in s for s in stats)

        bien = ok_pcm and ok_par and ok_pis and ok_stats
        fallas += not bien
        n_int = sum(int(e[2]) > 0 and any(int(t.split(",")[1]) & 1 for t in e[3:]) for e in enviado)
        print(f"{id_esc}: {nb} bloques | PCM {'OK' if ok_pcm else 'FALLA'} | PARAMS {'OK' if ok_par else 'FALLA'} "
              f"({n_int} con interferentes) | PISO {'OK' if ok_pis else 'FALLA'} (dif max {dif_max / 100:.2f} dB) | "
              f"parser {'OK' if ok_stats else 'FALLA: ' + ' '.join(stats)}")
        tasa = (len(open(tmp / "flujo.bin", "rb").read()) / (len(adc) / FS)) * 10 / 921600
        if id_esc == "P5":
            print(f"Uso de la UART a 921600 baud: {100 * tasa:.1f} %")
    print("RESULTADO:", "TODO OK" if not fallas else f"{fallas} escenario(s) con fallas")
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
