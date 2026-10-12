"""Reducción espectral por bloques (FFT/IFFT con solapamiento) para la sección 2.4.

Cadena por bloque: ventana de análisis -> FFT -> ganancia G[k] -> IFFT
-> ventana de síntesis -> suma solapada (overlap-add).
"""

import numpy as np
from scipy.io import wavfile

FS = 16000
N_BLOQUE = 1024


def leer_wav(ruta):
    """Lee un WAV PCM16 mono a FS y lo escala a [-1, 1] como el generador."""
    # Rechazar otro formato evita procesar sin querer una fs o un número de canales distinto.
    fs, datos = wavfile.read(ruta)
    if fs != FS or datos.dtype != np.int16 or datos.ndim != 1:
        raise ValueError(f'WAV incompatible (se espera PCM16 mono a {FS} Hz): {ruta}')
    return datos.astype(np.float64) / 32767


def es_potencia_de_dos(n):
    """Indica si n = 2^m con m >= 1."""
    # Un 2^m en binario tiene un solo bit en 1; n & (n-1) apaga ese bit.
    return isinstance(n, (int, np.integer)) and n >= 2 and (n & (n - 1)) == 0


def ventana_raiz_hann(n):
    """Raíz de la ventana Hann periódica de longitud n."""
    # Periódica (divide entre n, no n-1) para que los bloques al 50 % encajen exactamente.
    hann = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)
    # Se aplica dos veces (análisis y síntesis): el producto total vuelve a ser Hann.
    return np.sqrt(hann)


def ganancia_unitaria(espectro):
    """Ganancia neutra: no modifica ningún bin (sirve para probar la reconstrucción)."""
    return np.ones(len(espectro))


def procesar_bloques(entrada, n=N_BLOQUE, ganancia=ganancia_unitaria):
    """Aplica la cadena espectral por bloques de n muestras con 50 % de solapamiento.

    `ganancia` recibe el medio espectro X[0..n/2] de un bloque y devuelve un
    vector real G[0..n/2] con valores en [0, 1]. Devuelve una señal de la misma
    longitud que la entrada, alineada con ella (sin retardo).
    """
    # Validar evita bloques que no cumplen N = 2^m o entradas con varios canales.
    entrada = np.asarray(entrada, dtype=np.float64)
    if entrada.ndim != 1:
        raise ValueError('La entrada debe ser un vector unidimensional.')
    if not es_potencia_de_dos(n):
        raise ValueError(f'El tamaño de bloque debe ser potencia de dos, no {n}.')
    salto = n // 2
    ventana = ventana_raiz_hann(n)

    # Medio bloque de ceros al inicio: así las primeras muestras también quedan
    # cubiertas por dos ventanas y no salen atenuadas. Al final se rellena
    # hasta completar el último bloque.
    total = len(entrada) + 2 * salto
    total += (-(total - n)) % salto
    senal = np.zeros(total)
    senal[salto:salto + len(entrada)] = entrada
    salida = np.zeros(total)

    for inicio in range(0, total - n + 1, salto):
        bloque = senal[inicio:inicio + n] * ventana
        # rfft guarda solo los bins 0..n/2; los negativos son sus conjugados.
        espectro = np.fft.rfft(bloque)
        g = np.asarray(ganancia(espectro))
        # Una ganancia real por bin escala la magnitud sin tocar la fase y
        # mantiene X[n-k] = X*[k]; si fuera compleja, la salida dejaría de ser real.
        if g.shape != espectro.shape or np.iscomplexobj(g):
            raise ValueError('La ganancia debe ser real y de longitud n/2 + 1.')
        # irfft reconstruye la mitad negativa por simetría conjugada: salida real.
        recuperado = np.fft.irfft(g * espectro, n)
        # La ventana de síntesis suaviza los bordes que la modificación pudo alterar.
        salida[inicio:inicio + n] += recuperado * ventana
    return salida[salto:salto + len(entrada)]


def verificar_simetria(bloque, ganancia_medio):
    """Comprueba con la FFT completa que una ganancia simétrica da salida real.

    Construye G de longitud n con G[n-k] = G[k] a partir de G[0..n/2],
    aplica FFT -> G -> IFFT y devuelve el máximo |parte imaginaria|.
    """
    # Se usa fft/ifft completas a propósito: aquí nada fuerza la simetría,
    # así que una parte imaginaria ~1e-16 demuestra que se conservó.
    n = len(bloque)
    ganancia_completa = np.concatenate((ganancia_medio, ganancia_medio[1:n // 2][::-1]))
    salida = np.fft.ifft(ganancia_completa * np.fft.fft(bloque))
    return float(np.max(np.abs(salida.imag)))