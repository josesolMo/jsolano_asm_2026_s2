"""Diseno y procesamiento de filtros temporales FIR e IIR."""
from __future__ import annotations

import numpy as np
from scipy import signal as sg
from .config import FS, RIZADO_DB, ATEN_DB


def _medir(b, a, fp, fst):
    w, H = sg.freqz(b, a, worN=8192, fs=FS)
    m = np.abs(H)
    rizado = 20 * np.log10(m[w <= fp].max() / m[w <= fp].min())
    aten = -20 * np.log10(m[w >= fst].max())
    return rizado, aten


def disenar_fir(fp: float, fst: float) -> np.ndarray:
    """FIR pasa bajas de fase lineal (Parks-McClellan) del menor orden que cumple RIZADO_DB y ATEN_DB."""
    dp = (10 ** (RIZADO_DB / 20) - 1) / (10 ** (RIZADO_DB / 20) + 1)
    ds = 10 ** (-ATEN_DB / 20)
    for ntaps in range(11, 301, 2):                 # numero impar -> tipo I, retardo entero
        b = sg.remez(ntaps, [0, fp, fst, FS / 2], [1, 0], weight=[1 / dp, 1 / ds], fs=FS)
        rizado, aten = _medir(b, [1.0], fp, fst)
        if rizado <= RIZADO_DB and aten >= ATEN_DB:
            return b
    raise RuntimeError("No se encontro un FIR que cumpla la especificacion")


def disenar_iir(fp: float, fst: float) -> np.ndarray:
    """IIR eliptico pasa bajas, minimo orden (ellipord), en secciones de segundo orden (SOS)."""
    orden, wn = sg.ellipord(fp, fst, RIZADO_DB, ATEN_DB, fs=FS)
    return sg.ellip(orden, RIZADO_DB, ATEN_DB, wn, btype="low", output="sos", fs=FS)


def orden_iir(sos: np.ndarray) -> int:
    """Orden real del IIR: un orden impar rellena la ultima biquad con un polo en z = 0 que no cuenta."""
    return int(np.sum(np.abs(sg.sos2zpk(sos)[1]) > 1e-9))


class FiltroFIR:
    """y[n] = sum_{k=0}^{M} b[k] x[n-k]; conserva las ultimas M entradas entre bloques."""

    def __init__(self, b):
        self.b = b
        self.hist = np.zeros(len(b) - 1)

    def procesar(self, x: np.ndarray) -> np.ndarray:
        ext = np.concatenate([self.hist, x])
        self.hist = ext[len(ext) - len(self.b) + 1:]
        return np.convolve(ext, self.b, mode="valid")


class FiltroIIR:
    """Cascada de secciones biquad (forma directa II transpuesta); el estado pasa de bloque a bloque."""

    def __init__(self, sos):
        self.sos = sos
        self.zi = np.zeros((sos.shape[0], 2))

    def procesar(self, x: np.ndarray) -> np.ndarray:
        y, self.zi = sg.sosfilt(self.sos, x, zi=self.zi)
        return y
