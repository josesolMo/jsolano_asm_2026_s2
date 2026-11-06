"""Genera los audios de prueba P1-P5 (senal util + ruido controlado) para reproducirlos frente al
microfono del Nodo 1 desde un celular o la computadora. Usa los mismos escenarios y generadores que
la simulacion (sistema_ruido.scenarios / signals), asi lo que suena es lo que se simulo.

Uso, desde Proyecto_2/:  python full_system/generar_audios.py [--segundos 20]
Escribe en Proyecto_2/audios/:  P1_contaminada.wav ... P5_contaminada.wav  y  P1_limpia.wav ...
(16 kHz, 16 bits, mono). La version limpia sirve para comparar de oido con la salida del sistema.
"""
import argparse
import os
import sys
import wave
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
os.chdir(AQUI.parents[1])                       # config.py crea carpetas relativas a la raiz del repo
from sistema_ruido import scenarios, signals    # noqa: E402
from sistema_ruido.config import FS, PICO       # noqa: E402


def guardar(ruta, x):
    with wave.open(str(ruta), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(np.round(np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--segundos", type=float, default=20.0)
    a = ap.parse_args()
    salida = AQUI.parent / "audios"
    salida.mkdir(exist_ok=True)
    n = int(a.segundos * FS)
    t = np.arange(n) / FS
    for pid in ["P1", "P2", "P3", "P4", "P5"]:
        esc = scenarios.escenario(pid)
        rng = np.random.default_rng(esc["semilla"])
        limpia = signals.fade(signals.generar_senal(t=t, **esc["senal"]))
        ruido = signals.escalar_para_snr(limpia, signals.generar_ruido(n=n, rng=rng, **esc["ruido"]), esc["snr_db"])
        g = PICO / max(np.abs(limpia).max(), np.abs(limpia + ruido).max())
        guardar(salida / f"{pid}_contaminada.wav", g * (limpia + ruido))
        guardar(salida / f"{pid}_limpia.wav", g * limpia)
        print(f"{pid}: {esc['nombre']} | SNR {esc['snr_db']} dB | banda util {esc['banda_util']} Hz")
    print(f"Audios en {salida}")


if __name__ == "__main__":
    main()