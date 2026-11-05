"""Senales utiles, modelos de ruido, escalado SNR y cuantizacion ADC/DAC."""
from __future__ import annotations

import numpy as np
from scipy import signal as sg
from .config import FS, FADE_MS, F_RED


def rms(x) -> float:
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64))))


def fade(x: np.ndarray, ms: float = FADE_MS) -> np.ndarray:
    """Rampa sin^2 al inicio y final para evitar clics."""
    n = int(FS * ms / 1000)
    rampa = np.sin(np.linspace(0, np.pi / 2, n)) ** 2
    x = x.copy()
    x[:n] *= rampa
    x[-n:] *= rampa[::-1]
    return x


def generar_senal(tipo: str, t: np.ndarray, f=None, a=None, fase=None, notas=None) -> np.ndarray:
    """Senales utiles minimas de la seccion 2.1.

    'multitono'    : suma de senos (un tono es el caso de una sola frecuencia).
    'audio'        : melodia sintetica (notas con 6 armonicos y envolvente), sin voz de personas.
    'transitoria'  : rafagas sinusoidales que decaen, para ver la respuesta temporal.
    """
    if tipo == "multitono":
        return sum(ai * np.sin(2 * np.pi * fi * t + np.deg2rad(pi))
                   for fi, ai, pi in zip(f, a, fase))
    if tipo == "audio":
        seg = len(t) // len(notas)
        tau = np.arange(seg) / FS
        env = np.minimum(tau / 0.01, 1.0) * np.exp(-1.5 * tau)
        x = np.zeros(len(t))
        for i, f0 in enumerate(notas):
            x[i * seg:(i + 1) * seg] = env * sum(np.sin(2 * np.pi * k * f0 * tau) / k for k in range(1, 7))
        return x
    if tipo == "transitoria":
        env = np.exp(-5 * (t % 1.0)) * (np.sin(2 * np.pi * 2 * t) > 0)
        return env * np.sin(2 * np.pi * f[0] * t)
    raise ValueError(f"Senal no reconocida: {tipo}")


def generar_ruido(tipo: str, n: int, rng: np.random.Generator,
                  f: float = 2500.0, banda=(1000.0, 2200.0)) -> np.ndarray:
    """Las seis condiciones de la seccion 2.2.

    'tonal'     interferencia sinusoidal de frecuencia conocida f
    'red'       60 Hz y sus armonicos 2..4
    'hf'        ruido de alta frecuencia fuera de la banda util (banda)
    'blanco'    banda ancha 0-fs/2
    'banda'     banda limitada que se solapa parcialmente con la util (banda)
    'impulsivo' diez impulsos suaves de 0.5 a 4 ms
    """
    t = np.arange(n) / FS
    if tipo == "tonal":
        return np.sin(2 * np.pi * f * t + rng.uniform(-np.pi, np.pi))
    if tipo == "red":
        return sum(np.sin(2 * np.pi * F_RED * k * t + rng.uniform(-np.pi, np.pi)) / k for k in range(1, 5))
    if tipo == "blanco":
        return rng.standard_normal(n)
    if tipo in ("banda", "hf"):
        sos = sg.butter(6, banda, btype="bandpass", fs=FS, output="sos")
        return sg.sosfiltfilt(sos, rng.standard_normal(n))
    if tipo == "impulsivo":
        x = np.zeros(n)
        for loc in rng.choice(np.arange(int(.1 * FS), n - int(.1 * FS)), size=10, replace=False):
            ancho = max(2, int(rng.uniform(.0005, .004) * FS))
            fin = min(n, loc + ancho)
            x[loc:fin] += rng.choice([-1, 1]) * np.hanning(2 * (fin - loc))[:fin - loc]
        return x
    raise ValueError(f"Ruido no reconocido: {tipo}")


def escalar_para_snr(limpia: np.ndarray, ruido: np.ndarray, snr_db: float) -> np.ndarray:
    """Escala el ruido para que SNR = 10log10(P_senal / P_ruido) sea snr_db (por RMS)."""
    return ruido * (rms(limpia) / 10 ** (snr_db / 20)) / (rms(ruido) + 1e-15)


def cuantizar(x: np.ndarray, bits: int) -> np.ndarray:
    """ADC/DAC ideal de 'bits' bits en [-1, 1): redondeo al nivel mas cercano y saturacion."""
    paso = 2.0 / 2 ** bits
    return np.clip(np.round(x / paso) * paso, -1.0, 1.0 - paso)
