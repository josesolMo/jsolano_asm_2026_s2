"""Metricas de calidad y estimacion de costo computacional."""
from __future__ import annotations

import numpy as np
from scipy import signal as sg
from .config import FS, N, L, LAG_MAX, HOP, K_MEDIANA


def potencia_en_rango(x: np.ndarray, lo: float, hi: float) -> float:
    """Potencia media de x entre lo y hi Hz (Parseval sobre la FFT completa)."""
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / FS)
    P = np.abs(X) ** 2
    P[1:] *= 2
    return float(P[(f >= lo) & (f <= hi)].sum() / len(x) ** 2)


def alinear(s: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, int]:
    """Desplaza y el entero en [-LAG_MAX, LAG_MAX] que mas se parece a s (compensa el retardo de grupo)."""
    sl = slice(N, L - N)
    lag = max(range(-LAG_MAX, LAG_MAX + 1), key=lambda k: np.dot(s[sl], np.roll(y, -k)[sl]))
    return np.roll(y, -lag), lag


def metricas(s: np.ndarray, x: np.ndarray, y: np.ndarray, esc: dict) -> dict:
    """MSE, SNR entrada/salida, mejora, energia util conservada y atenuacion de la interferencia.

    Se descartan N muestras en cada extremo (transitorio de arranque/cierre).
      MSE  = mean((s - s_hat)^2)                       ec. (2)
      SNR  = 10 log10( sum s^2 / sum (s - s_hat)^2 )   ec. (3)
      E_util = 100 * (<s_hat, s> / <s, s>)^2           proyeccion de la salida sobre la referencia:
                                                       100 % = la senal util pasa sin perdida; puede
                                                       superar 100 % por el rizado de la banda de paso
      Atenuacion = 10 log10( P_rango(entrada) / P_rango(salida) )
    """
    sl = slice(N, L - N)
    ya, lag = alinear(s, y)
    snr = lambda e: 10 * np.log10(np.sum(s[sl] ** 2) / np.sum((s[sl] - e[sl]) ** 2))
    g = np.dot(ya[sl], s[sl]) / np.dot(s[sl], s[sl])
    lo, hi = esc["rango_int"]
    return dict(mse=float(np.mean((s[sl] - ya[sl]) ** 2)), snr_in=float(snr(x)), snr_out=float(snr(ya)),
                dsnr=float(snr(ya) - snr(x)), e_util=float(100 * g ** 2),
                atenuacion=float(10 * np.log10(potencia_en_rango(x, lo, hi) / potencia_en_rango(y, lo, hi))),
                retardo=lag)


def costo_estimado(b: np.ndarray, sos: np.ndarray) -> list[dict]:
    """Operaciones por bloque de N muestras, memoria (float32) y latencia, aproximadas.

    Supuestos: FFT real ~ 2.5 N log2(N) flops; biquad ~ 5 MAC/muestra; latencia = captura del
    bloque + retardo algoritmico (sin tiempo de computo, que depende del microcontrolador).
    """
    M, S, fft = len(b) - 1, sos.shape[0], 2.5 * N * np.log2(N)
    nb = N // 2 + 1
    b_iir, a_iir = sg.sos2tf(sos)
    f_paso = np.linspace(0, 1000, 64)
    gd_iir = float(np.mean(sg.group_delay((b_iir, a_iir), w=f_paso, fs=FS)[1]))
    filas = [
        dict(nodo="Nodo 1 (DC+ventana+FFT+diag.)", mflops=(2 * N + fft + 4 * nb + nb * K_MEDIANA) / (N / FS) / 1e6,
             mem_B=4 * (N + nb + nb + N + 2) + 2 * 2 * N, lat_ms=1e3 * N / FS),
        dict(nodo=f"Nodo 2 FIR (orden {M})", mflops=2 * (M + 1) * N / (N / FS) / 1e6,
             mem_B=4 * (2 * M + 1) + 2 * 2 * N, lat_ms=1e3 * (N + M / 2) / FS),
        dict(nodo=f"Nodo 2 IIR ({S} biquads)", mflops=2 * 5 * S * N / (N / FS) / 1e6,
             mem_B=4 * S * 7 + 2 * 2 * N, lat_ms=1e3 * (N + gd_iir) / FS),
        dict(nodo="Nodo 3 espectral (FFT+IFFT)", mflops=2 * (2 * fft + 2 * N + 10 * nb) / (N / FS) / 1e6,
             mem_B=4 * (HOP + 2 * HOP + (N + 2) + 2 * nb), lat_ms=1e3 * (N + HOP) / FS),
    ]
    return filas
