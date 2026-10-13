"""Reducción espectral por bloques (FFT/IFFT con solapamiento) para la sección 2.4.

Cadena por bloque: ventana de análisis -> FFT -> ganancia G[k] -> IFFT
-> ventana de síntesis -> suma solapada (overlap-add).
"""

import numpy as np
from scipy.io import wavfile
from scipy.ndimage import median_filter

FS = 16000
N_BLOQUE = 1024
ALFA = 6.0          # Factor de sobresustracción: cuántas veces el ruido medio se resta.
G_MIN = 0.05        # Ganancia mínima (-26 dB): deja un residuo en vez de huecos de silencio.
ANCHO_MEDIANA = 31  # Bins de la mediana local (~484 Hz con N=1024 a 16 kHz).
BANDA_UTIL = (340.0, 1420.0)  # Hz; de metadatos/configuraciones.json (igual en los 3 casos).
UMBRAL_PICO_DB = 20.0  # Cuánto debe sobresalir un pico del ruido local para ser interferencia.
ANCHO_MUESCA = 4       # Bins atenuados a cada lado del pico (±62.5 Hz con N=1024).
G_MUESCA = 0.0         # Ganancia dentro de la muesca.


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


def estimar_potencia_ruido(espectro, ancho=ANCHO_MEDIANA):
    """Estima, bin por bin, la potencia media del ruido de un bloque.

    Usa la mediana de |X|^2 en una vecindad de `ancho` bins alrededor de cada bin.
    """
    if ancho < 1 or ancho % 2 == 0:
        raise ValueError('El ancho de la mediana debe ser un entero impar positivo.')
    potencia = np.abs(espectro) ** 2
    # La mediana ignora los pocos bins ocupados por tonos, así que sigue a la "alfombra".
    # Es local (no global) porque el ruido puede no ser plano, como la banda del caso 3.
    mediana = median_filter(potencia, size=ancho, mode='reflect')
    # Para ruido gaussiano |X|^2 sigue una exponencial, cuya mediana es ln(2) veces la media.
    return mediana / np.log(2)


def ganancia_sustraccion(espectro, alfa=ALFA, g_min=G_MIN, ancho=ANCHO_MEDIANA,
                         potencia_ruido=None):
    """Ganancia de sustracción espectral adaptativa para un bloque.

    G[k] = sqrt(max(1 - alfa * Pruido[k] / |X[k]|^2, g_min^2)).
    Si `potencia_ruido` es None, se estima del propio bloque (parámetro adaptativo);
    si se recibe (por ejemplo del nodo 1), se usa esa estimación.
    """
    potencia = np.abs(espectro) ** 2
    if potencia_ruido is None:
        potencia_ruido = estimar_potencia_ruido(espectro, ancho)
    # Se resta alfa veces el ruido estimado a la potencia de cada bin: los bins que son
    # casi solo ruido quedan cerca de 0 y los tonos, muy por encima, quedan cerca de 1.
    # El 1e-30 evita dividir entre cero en bins totalmente vacíos.
    restante = 1 - alfa * potencia_ruido / np.maximum(potencia, 1e-30)
    # La raíz convierte la fracción de potencia en ganancia de amplitud.
    return np.sqrt(np.maximum(restante, g_min ** 2))


def detectar_picos(espectro, n, banda_util=BANDA_UTIL, umbral_db=UMBRAL_PICO_DB,
                   ancho=ANCHO_MEDIANA):
    """Devuelve los bins de picos interferentes: máximos locales fuera de la banda útil
    que superan en `umbral_db` la potencia de ruido estimada en su vecindad."""
    potencia = np.abs(espectro) ** 2
    frecuencias = np.arange(len(espectro)) * FS / n
    fuera = (frecuencias < banda_util[0]) | (frecuencias > banda_util[1])
    # Máximo local: mayor o igual que sus dos vecinos; así se marca solo la cima del pico.
    maximo = np.zeros(len(potencia), dtype=bool)
    maximo[1:-1] = (potencia[1:-1] >= potencia[:-2]) & (potencia[1:-1] >= potencia[2:])
    # 20 dB = 100 veces el ruido medio. En ruido gaussiano un bin supera eso con
    # probabilidad e^-100, así que la alfombra nunca dispara la detección.
    sobresale = potencia > 10 ** (umbral_db / 10) * estimar_potencia_ruido(espectro, ancho)
    return np.flatnonzero(maximo & fuera & sobresale)


def aplicar_muescas(ganancia, picos, ancho=ANCHO_MUESCA, g_muesca=G_MUESCA):
    """Baja la ganancia a g_muesca en ±ancho bins alrededor de cada pico."""
    ganancia = ganancia.copy()
    for k in picos:
        # Se cubre el lóbulo principal y los primeros laterales de la ventana, donde
        # el tono también deja energía; con un solo bin quedaría casi todo su residuo.
        zona = slice(max(k - ancho, 0), k + ancho + 1)
        ganancia[zona] = np.minimum(ganancia[zona], g_muesca)
    return ganancia


def snr_db(referencia, salida):
    """SNR de la salida respecto a la referencia limpia, en dB."""
    referencia = np.asarray(referencia, dtype=np.float64)
    error = referencia - np.asarray(salida, dtype=np.float64)
    return float(10 * np.log10(np.sum(referencia ** 2) / np.sum(error ** 2)))


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


def reducir_ruido(entrada, n=N_BLOQUE, alfa=ALFA, g_min=G_MIN, ancho=ANCHO_MEDIANA,
                  banda_util=BANDA_UTIL, umbral_db=UMBRAL_PICO_DB, registro=None):
    """Reducción espectral completa: sustracción adaptativa + muescas en picos interferentes.

    Si `registro` es una lista, se le agrega por bloque un arreglo con las
    frecuencias (Hz) de los picos detectados.
    """
    def ganancia(espectro):
        g = ganancia_sustraccion(espectro, alfa, g_min, ancho)
        picos = detectar_picos(espectro, n, banda_util, umbral_db, ancho)
        if registro is not None:
            registro.append(picos * FS / n)
        return aplicar_muescas(g, picos)

    return procesar_bloques(entrada, n, ganancia)


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