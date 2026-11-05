"""Ejecucion de una prueba de extremo a extremo a traves de los tres nodos."""
from __future__ import annotations

import numpy as np
from .config import FS, N, HOP, L, N_BLOQUES, ADC_BITS, DAC_BITS, TRANS_HZ, PICO
from .signals import fade, generar_senal, generar_ruido, escalar_para_snr, cuantizar
from .acquisition import NodoAdquisicion
from .temporal_filters import disenar_fir, disenar_iir, FiltroFIR, FiltroIIR
from .spectral import NodoEspectral
from .metrics import metricas


def correr_prueba(esc: dict) -> dict:
    """Genera la senal, la pasa por los tres nodos bloque a bloque y calcula las metricas."""
    rng = np.random.default_rng(esc["semilla"])
    t = np.arange(L) / FS
    limpia = fade(generar_senal(t=t, **esc["senal"]))
    ruido = escalar_para_snr(limpia, generar_ruido(n=L, rng=rng, **esc["ruido"]), esc["snr_db"])
    ganancia = PICO / max(np.abs(limpia).max(), np.abs(limpia + ruido).max())
    limpia, ruidosa = ganancia * limpia, ganancia * (limpia + ruido)
    adc = cuantizar(ruidosa, ADC_BITS)

    fp = esc["banda_util"][1]
    fst = fp + TRANS_HZ
    b, sos = disenar_fir(fp, fst), disenar_iir(fp, fst)
    nodo1, fir, iir, esp = NodoAdquisicion(esc["banda_util"]), FiltroFIR(b), FiltroIIR(sos), NodoEspectral()

    y_fir, y_iir, y_esp, diag = [], [], [], None
    for i in range(N_BLOQUES):
        bloque = adc[i * N:(i + 1) * N]
        diag = nodo1.procesar(bloque)                                   # Nodo 1 -> diagnostico
        y_fir.append(fir.procesar(bloque))                              # Nodo 2 (FIR)
        y_iir.append(iir.procesar(bloque))                              # Nodo 2 (IIR candidato)
        for h in range(2):                                              # Nodo 3
            y_esp.append(esp.avanzar(bloque[h * HOP:(h + 1) * HOP], diag))
    y_esp.append(esp.avanzar(np.zeros(HOP), diag))                      # vaciar la cola de solape
    salidas = {"FIR": np.concatenate(y_fir), "IIR": np.concatenate(y_iir),
               "ESPECTRAL": np.concatenate(y_esp)[HOP:HOP + L]}
    salidas = {k: cuantizar(v, DAC_BITS) for k, v in salidas.items()}   # salida por DAC

    return dict(esc=esc, limpia=limpia, adc=adc, salidas=salidas, diag=diag, b=b, sos=sos,
                metricas={k: metricas(limpia, adc, v, esc) for k, v in salidas.items()})
