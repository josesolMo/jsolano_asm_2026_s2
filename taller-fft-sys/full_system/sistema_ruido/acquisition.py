"""Nodo 1: adquisicion, FFT y diagnostico espectral."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from scipy import signal as sg
from scipy.ndimage import median_filter
from .config import N, LAMBDA_EMA, K_MEDIANA, UMBRAL_PICO_DB, RANGO_PICO_DB, BIN_HZ, GUARDA_BINS, F_RED


@dataclass
class Diagnostico:
    """Parametros que el Nodo 1 comunica a los Nodos 2 y 3."""
    piso: np.ndarray      # PSD de ruido estimada por bin (misma normalizacion que el periodograma)
    picos: np.ndarray     # bins de picos estrechos
    interf: np.ndarray    # bins de picos clasificados como interferencia
    psd: np.ndarray       # espectro promediado (para graficar)


class NodoAdquisicion:
    """ADC -> quitar DC -> ventana Hann -> FFT -> piso de ruido, picos e interferentes.

    El espectro se promedia con una EMA para estabilizar la estimacion. El piso se
    obtiene con un filtro de mediana sobre la frecuencia: los picos estrechos no lo
    afectan, pero un ruido de banda ancha o de banda limitada si.
    """

    def __init__(self, banda_util):
        self.banda = banda_util
        self.w = sg.get_window("hann", N)
        self.psd = None

    def procesar(self, bloque: np.ndarray) -> Diagnostico:
        x = bloque - bloque.mean()                                   # elimina DC
        P = np.abs(np.fft.rfft(x * self.w)) ** 2 / np.sum(self.w ** 2)
        self.psd = P if self.psd is None else LAMBDA_EMA * self.psd + (1 - LAMBDA_EMA) * P
        piso = median_filter(self.psd, size=K_MEDIANA, mode="reflect")

        # picos: maximos locales sobre el piso y no mas de RANGO_PICO_DB bajo el mayor
        mx = np.flatnonzero((self.psd[1:-1] > self.psd[:-2]) & (self.psd[1:-1] >= self.psd[2:])) + 1
        picos = mx[self.psd[mx] > piso[mx] * 10 ** (UMBRAL_PICO_DB / 10)]
        if picos.size:
            picos = picos[self.psd[picos] >= self.psd[picos].max() * 10 ** (-RANGO_PICO_DB / 10)]

        # interferente = fuera de la banda util (con guarda) o zumbido de red. Se declara zumbido
        # solo si esta el fundamental de 60 Hz y al menos otro armonico (k = 2..8): un pico aislado
        # sobre un multiplo de 60 Hz puede ser un tono util (p. ej. 420 Hz = 7 x 60).
        f = picos * BIN_HZ
        lo, hi = self.banda
        fuera = (f < lo - GUARDA_BINS * BIN_HZ) | (f > hi + GUARDA_BINS * BIN_HZ)
        k = np.round(f / F_RED)
        red = (k >= 1) & (k <= 8) & (np.abs(f - k * F_RED) < 0.75 * BIN_HZ)
        red = red & (red.sum() >= 2) & bool(np.any(red & (k == 1)))
        return Diagnostico(piso, picos, picos[fuera | red], self.psd.copy())
