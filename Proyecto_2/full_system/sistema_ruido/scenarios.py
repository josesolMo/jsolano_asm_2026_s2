"""Escenarios P1-P6 y validacion del plan de frecuencias."""
from __future__ import annotations

import numpy as np
from .config import FS, TRANS_HZ, BIN_HZ, ADC_BITS


TRES_TONOS = dict(tipo="multitono", f=[440, 880, 1320], a=[1.0, 0.55, 0.30], fase=[0, 35, -50])


def escenario(pid: str, rep: int = 0) -> dict:
    """Configuracion de una prueba de la Tabla 1. 'rep' cambia la semilla (repeticiones).

    Campos clave
      banda_util : (lo, hi) Hz que el sistema debe conservar. Es un parametro de
                   tiempo de ejecucion: en la defensa se ajusta sin recompilar.
      rango_int  : (lo, hi) Hz donde vive la interferencia principal (para medir su atenuacion).
      solape     : True si util e interferencia comparten frecuencias A PROPOSITO.
    """
    semilla = 2026_000 + 100 * int(pid[1]) + rep
    if pid == "P1":
        return dict(id=pid, nombre="Tono 1 kHz + interferencia sinusoidal 2.5 kHz", semilla=semilla, snr_db=5.0,
                    senal=dict(tipo="multitono", f=[1000], a=[1.0], fase=[0]),
                    ruido=dict(tipo="tonal", f=2500.0), banda_util=(900, 1100), rango_int=(2485, 2515), solape=False)
    if pid == "P2":
        return dict(id=pid, nombre="Tres tonos + ruido de alta frecuencia 4.5-7.5 kHz", semilla=semilla, snr_db=0.0,
                    senal=TRES_TONOS, ruido=dict(tipo="hf", banda=(4500, 7500)),
                    banda_util=(340, 1420), rango_int=(4500, 7500), solape=False)
    if pid == "P3":
        return dict(id=pid, nombre="Audio sintetico + ruido blanco", semilla=semilla, snr_db=3.0,
                    senal=dict(tipo="audio", notas=[220.0, 261.63, 329.63, 440.0]), ruido=dict(tipo="blanco"),
                    banda_util=(150, 2700), rango_int=(2700 + TRANS_HZ, FS / 2), solape=False)
    if pid == "P4":
        return dict(id=pid, nombre="Transitoria 600 Hz + ruido impulsivo", semilla=semilla, snr_db=5.0,
                    senal=dict(tipo="transitoria", f=[600]), ruido=dict(tipo="impulsivo"),
                    banda_util=(300, 900), rango_int=(900 + TRANS_HZ, FS / 2), solape=False)
    if pid == "P5":
        return dict(id=pid, nombre="Tres tonos + banda limitada 1-2.2 kHz (solapa 1320 Hz)", semilla=semilla, snr_db=0.0,
                    senal=TRES_TONOS, ruido=dict(tipo="banda", banda=(1000, 2200)),
                    banda_util=(340, 1420), rango_int=(1000, 2200), solape=True)
    if pid == "P6":   # senal y ruido sorteados: modela la prueba desconocida de la defensa
        rng = np.random.default_rng(semilla)
        kind = ["uno", "tres", "audio"][rep % 3]
        if kind == "uno":
            f = [float(rng.integers(300, 1500))]
            senal, banda = dict(tipo="multitono", f=f, a=[1.0], fase=[0]), (f[0] - 100, f[0] + 100)
        elif kind == "tres":
            f = sorted(float(v) for v in rng.integers(300, 1600, 3))
            senal, banda = dict(tipo="multitono", f=f, a=[1.0, .6, .4], fase=[0, 20, -40]), (f[0] - 100, f[-1] + 100)
        else:
            senal, banda = dict(tipo="audio", notas=[196.0, 246.94, 293.66, 392.0]), (120, 2400)
        tipo = str(rng.choice(["tonal", "red", "hf", "blanco", "banda", "impulsivo"]))
        ruido, solape = dict(tipo=tipo), False
        if tipo == "tonal":
            ruido["f"] = float(rng.integers(int(banda[1]) + 600, 3800))
            rango = (ruido["f"] - 15, ruido["f"] + 15)
        elif tipo == "red":
            rango, solape = (30, 270), banda[0] < 270
        elif tipo == "hf":
            ruido["banda"] = (4200, 7500)
            rango = ruido["banda"]
        elif tipo == "banda":
            lo = float(rng.integers(int(banda[1]) - 300, int(banda[1]) + 100))
            ruido["banda"] = (lo, lo + 1200)
            rango, solape = ruido["banda"], lo < banda[1]
        else:
            rango = (banda[1] + TRANS_HZ, FS / 2)
        return dict(id=pid, nombre=f"Desconocida: {kind} + ruido {tipo}", semilla=semilla,
                    snr_db=float(rng.uniform(-2, 8)), senal=senal, ruido=ruido,
                    banda_util=banda, rango_int=rango, solape=solape)
    raise ValueError(pid)


def separacion_hz(a, b) -> float:
    """Distancia entre dos intervalos de frecuencia (0 si se solapan)."""
    return max(0.0, max(a[0], b[0]) - min(a[1], b[1]))


def validar_frecuencias(esc: dict) -> list[str]:
    """Criterios de diseno del plan de frecuencias. Devuelve advertencias (vacio = cumple)."""
    av = []
    lo, hi = esc["banda_util"]
    ri = esc["rango_int"]
    puntual = esc["ruido"]["tipo"] in ("tonal", "hf", "banda")   # el ruido de banda ancha llega a Nyquist por definicion
    if max(hi, ri[1] if puntual else 0) >= 0.95 * FS / 2:
        av.append("componente demasiado cerca de Nyquist")
    if lo < 3 * BIN_HZ:
        av.append("banda util muy cerca de DC (fuga de la ventana)")
    sep = separacion_hz((lo, hi), ri)
    if sep == 0 and not esc["solape"]:
        av.append("solape no previsto entre util e interferencia")
    if 0 < sep < 4 * BIN_HZ:
        av.append(f"separacion {sep:.0f} Hz < 4 bins: dentro del lobulo principal de la ventana")
    return av


def chequeo_aliasing() -> dict:
    """Justifica fs: Nyquist, frecuencia que se pliega sobre la banda util y filtro analogico necesario."""
    f_max_util = max(esc_max_freq())
    f_pliega = FS - f_max_util                 # la componente mas baja que cae sobre f_max_util
    req_db = 6.02 * ADC_BITS + 1.76            # rango dinamico del ADC ideal
    rc_fc = FS / 4
    rc_db = 10 * np.log10(1 + (f_pliega / rc_fc) ** 2)   # RC de 1er orden con fc = fs/4
    return dict(nyquist=FS / 2, f_max_util=f_max_util, f_pliega=f_pliega,
                atenuacion_requerida_db=req_db, rc_1er_orden_db=float(rc_db),
                ejemplo_9k_se_pliega_en=float(abs(9000 - FS * round(9000 / FS))))


def esc_max_freq() -> list[float]:
    """Frecuencia util mas alta entre las pruebas P1-P5."""
    return [escenario(p)["banda_util"][1] for p in ("P1", "P2", "P3", "P4", "P5")]
