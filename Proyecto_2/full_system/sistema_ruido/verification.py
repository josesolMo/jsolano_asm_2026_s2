"""Pruebas internas de equivalencia, reconstruccion, CRC y resumen de filtros."""
from __future__ import annotations

import numpy as np
from scipy import signal as sg
from .config import N, N_BLOQUES, HOP, L, TRANS_HZ, FS
from .temporal_filters import FiltroFIR, FiltroIIR, _medir, orden_iir
from .spectral import NodoEspectral
from .link import crc16


def dft_directa(x: np.ndarray) -> np.ndarray:
    """DFT por definicion X[k] = sum x[n] exp(-j 2 pi k n / N), O(N^2)."""
    n = np.arange(len(x))
    return np.exp(-2j * np.pi * n[:, None] * n / len(x)) @ x


def verificar_bloques(res: dict) -> None:
    rng = np.random.default_rng(0)
    x = rng.standard_normal(N)
    err_dft = np.max(np.abs(dft_directa(x) - np.fft.fft(x)))
    assert err_dft < 1e-9, "DFT directa != FFT"

    # procesar por bloques con estado == procesar toda la senal de una vez
    adc, b, sos = res["adc"], res["b"], res["sos"]
    fir, iir = FiltroFIR(b), FiltroIIR(sos)
    y1 = np.concatenate([fir.procesar(adc[i * N:(i + 1) * N]) for i in range(N_BLOQUES)])
    y2 = np.concatenate([iir.procesar(adc[i * N:(i + 1) * N]) for i in range(N_BLOQUES)])
    assert np.allclose(y1, sg.lfilter(b, [1.0], adc)), "FIR por bloques != FIR continuo"
    assert np.allclose(y2, sg.sosfilt(sos, adc)), "IIR por bloques != IIR continuo"

    # el nodo espectral con ganancia 1 debe devolver la entrada (reconstruccion perfecta)
    esp, x_in = NodoEspectral(), rng.standard_normal(L)
    sal = [esp.avanzar(x_in[i * HOP:(i + 1) * HOP], None) for i in range(L // HOP)]
    sal.append(esp.avanzar(np.zeros(HOP), None))
    err_wola = np.max(np.abs(np.concatenate(sal)[HOP:HOP + L] - x_in))
    assert err_wola < 1e-12, "sqrt-Hann + 50 % no reconstruye"
    assert crc16(b"123456789") == 0x29B1, "CRC-16 incorrecto"
    print(f"Verificaciones OK: |DFT-FFT|={err_dft:.1e}, bloques==continuo, reconstruccion WOLA={err_wola:.1e}, CRC-16")


def resumen_filtros(res: dict) -> dict:
    """Orden, rizado, atenuacion, estabilidad y efecto de cuantizar coeficientes."""
    b, sos, esc = res["b"], res["sos"], res["esc"]
    fp = esc["banda_util"][1]
    fst = fp + TRANS_HZ
    z, p, _ = sg.sos2zpk(sos)
    p32 = sg.sos2zpk(sos.astype(np.float32).astype(np.float64))[1]
    bq = np.round(b * 2 ** 15) / 2 ** 15                   # coeficientes en Q15
    b_i, a_i = sg.sos2tf(sos)
    return dict(fir=dict(orden=len(b) - 1, retardo_muestras=(len(b) - 1) // 2,
                         rizado_db=_medir(b, [1.0], fp, fst)[0], aten_db=_medir(b, [1.0], fp, fst)[1],
                         aten_q15_db=_medir(bq, [1.0], fp, fst)[1], max_coef=float(np.abs(b).max())),
                iir=dict(orden=orden_iir(sos), biquads=int(sos.shape[0]),
                         rizado_db=_medir(b_i, a_i, fp, fst)[0], aten_db=_medir(b_i, a_i, fp, fst)[1],
                         radio_polo_max=float(np.abs(p).max()), radio_polo_max_float32=float(np.abs(p32).max()),
                         max_coef=float(np.abs(sos[:, [0, 1, 2, 4, 5]]).max())))
