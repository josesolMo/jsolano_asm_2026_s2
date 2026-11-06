"""Nodo 3: filtrado espectral adaptativo con solape y suma."""
from __future__ import annotations

import numpy as np
from scipy import signal as sg
from .config import ALFA, BETA, N, HOP, NOTCH_BINS, G_INTERF
from .acquisition import Diagnostico


class NodoEspectral:
    """Sustraccion espectral adaptativa con analisis-sintesis sqrt-Hann y 50 % de solape.

    Por cada avance de HOP muestras:
      1. ventana sqrt-Hann sobre las ultimas N muestras   2. rfft
      3. ganancia real por bin  G = sqrt(clip(1 - ALFA*piso/P, BETA^2, 1))
         (piso viene del Nodo 1; ALFA es el parametro adaptativo)
      4. G = G_INTERF en +-NOTCH_BINS alrededor de cada interferente detectado
      5. irfft, ventana de sintesis y suma con solape (overlap-add)

    La ganancia es real y se aplica sobre rfft/irfft: la simetria conjugada del
    espectro se conserva automaticamente y la salida es real. Como sqrt-Hann al
    cuadrado suma 1 con 50 % de solape, ganancia 1 reconstruye la entrada exacta.
    """

    def __init__(self, alfa: float = ALFA, beta: float = BETA):
        self.alfa, self.beta = alfa, beta
        self.w = np.sqrt(sg.get_window("hann", N))
        self.norma = np.sum(self.w ** 2)
        self.prev = np.zeros(HOP)
        self.ola = np.zeros(HOP)

    def ganancia(self, P: np.ndarray, diag: Diagnostico) -> np.ndarray:
        G = np.sqrt(np.clip(1 - self.alfa * diag.piso / (P + 1e-20), self.beta ** 2, 1.0))
        for b in diag.interf:
            G[max(0, b - NOTCH_BINS): b + NOTCH_BINS + 1] = G_INTERF
        return G

    def avanzar(self, nuevo: np.ndarray, diag: Diagnostico | None) -> np.ndarray:
        """Consume HOP muestras y entrega HOP muestras (con retardo de HOP). diag=None => G=1."""
        X = np.fft.rfft(np.concatenate([self.prev, nuevo]) * self.w)
        self.prev = nuevo
        G = 1.0 if diag is None else self.ganancia(np.abs(X) ** 2 / self.norma, diag)
        y = np.fft.irfft(X * G, N) * self.w
        salida = self.ola + y[:HOP]
        self.ola = y[HOP:]
        return salida
