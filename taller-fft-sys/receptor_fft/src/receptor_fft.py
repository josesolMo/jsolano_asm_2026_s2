"""Receptor del nodo 1: identifica frecuencias en la FFT y decide si se aceptan.

Referencia en Python de la estrategia de recepción. No es el firmware: describe,
paso a paso, lo que el ESP32-S3 debe hacer con cada bloque para que el resultado
pueda compararse contra esta implementación.

Cadena por bloque:
    bloque ADC -> quitar DC -> ventana Hann -> FFT -> |X|^2 -> piso de ruido
    -> candidatos (máximos locales) -> criterios instantáneos C1..C3
    -> seguimiento entre bloques (C4, persistencia) -> clasificación -> trama.
"""

from collections import deque

import numpy as np
from scipy.ndimage import percentile_filter

# Parámetros comunes con la Tarea 1 y la propuesta 2.5 (ESP32-S3).
FS = 16000          # Hz
N_BLOQUE = 1024     # muestras por bloque (2^10), df = 15.625 Hz
SALTO = 512         # 50 % de solapamiento: un análisis cada 32 ms

# Criterios de aceptación (fijados antes de las pruebas finales, sección 3.5).
UMBRAL_DB = 20.0     # C1: altura mínima del pico sobre el piso de ruido local
ANCHO_PISO = 31      # bins usados para estimar el piso (~484 Hz)
PERCENTIL_PISO = 25  # percentil del piso; ver piso_ruido()
K_MIN = 3            # C2: primer bin válido (46.9 Hz); por debajo domina la fuga del DC
MARGEN_NYQUIST = 2   # C2: bins descartados junto a fs/2
ANCHO_MAX_BINS = 3   # C3: ancho máximo a -6 dB de un tono con Hann (teórico: 2 bins)
TOLERANCIA_BINS = 1.5  # C4: cuánto puede moverse una frecuencia entre bloques
M_HISTORIA = 4       # C4: bloques recordados por cada frecuencia
K_ACEPTAR = 3        # C4: se acepta con 3 de los últimos 4 bloques
K_LIBERAR = 1        # histéresis: se libera con 1 o menos de los últimos 4
MAX_REPORTADAS = 8   # límite de frecuencias por trama (39 bytes)
BANDA_UTIL = (340.0, 1420.0)  # Hz; configurable desde la interfaz
# C6: frecuencias que el emisor tiene configuradas (taller semana 12). Se toman las
# de la Tarea 1; el emisor real las comunica o se cargan por UART.
FRECUENCIAS_EMISOR = (440.0, 880.0, 1320.0, 2500.0)
# Tabla por modo del emisor real (emisor_esp32/senales.h, comandos 1 a 4).
TABLAS_POR_MODO = {
    1: (1000.0,),               # seno de 1000 Hz
    2: (2500.0,),               # seno de 2500 Hz (interferente individual)
    3: (440.0, 880.0, 1320.0),  # multitono
    4: (600.0,),                # transitoria: rafagas de 600 Hz que decaen
}
TOLERANCIA_EMISOR_HZ = FS / N_BLOQUE  # ±1 bin: 60 veces el error de la interpolación
# C6: una frecuencia ajena a la tabla solo se reporta si está a menos de 40 dB del tono
# más fuerte (mismo rango que RANGO_PICO_DB de full_system). Más abajo están los
# armónicos de cuantización del DAC de 8 bits (~ -48 dB), que no son interferencias.
RANGO_RELATIVO_DB = 40.0

# Banderas de la trama de parámetros.
FLAG_INTERFERENTE = 0x01
FLAG_ARMONICO = 0x02
FLAG_BANDA_UTIL = 0x04


def ventana_hann(n):
    """Ventana Hann periódica (la misma familia que usa el nodo espectral)."""
    return 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)


def espectro_potencia(bloque):
    """Quita el DC, aplica Hann y devuelve |X[k]|^2 para k = 0..N/2."""
    bloque = np.asarray(bloque, dtype=np.float64)
    n = len(bloque)
    if n < 8 or n & (n - 1):
        raise ValueError(f'El bloque debe tener 2^m muestras, no {n}.')
    # El ADC del ESP32 entrega valores sin signo con un nivel medio; si no se resta,
    # el bin 0 y su fuga tapan las frecuencias bajas (por ejemplo 50/60 Hz).
    sin_dc = bloque - np.mean(bloque)
    return np.abs(np.fft.rfft(sin_dc * ventana_hann(n))) ** 2


def piso_ruido(potencia, ancho=ANCHO_PISO, percentil=PERCENTIL_PISO):
    """Potencia media de ruido por bin a partir de un percentil local de |X|^2.

    En ruido gaussiano |X|^2 es exponencial: su percentil q vale -ln(1 - q) veces
    la media, así que se divide entre ese factor para obtener la media.
    """
    # Se usa el percentil 25 y no la mediana: con 60 Hz, sus armónicos y los tonos
    # útiles, más de la mitad de los 31 bins de baja frecuencia pueden ser lóbulos
    # de tonos; la mediana sube el piso y esconde armónicos reales.
    q = percentil / 100
    return percentile_filter(potencia, percentil, size=ancho, mode='reflect') / -np.log(1 - q)


def interpolar_pico(potencia_db, k):
    """Interpolación parabólica sobre los dB de k-1, k, k+1.

    Devuelve (desplazamiento en bins, altura del pico en dB). Con Hann el error
    queda muy por debajo de medio bin, que es el error sin interpolar.
    """
    a, b, c = potencia_db[k - 1], potencia_db[k], potencia_db[k + 1]
    den = a - 2 * b + c
    # den = 0 solo si los tres puntos están alineados; entonces no hay vértice útil.
    d = 0.0 if den == 0 else 0.5 * (a - c) / den
    return d, b - 0.25 * (a - c) * d


def ancho_6db(potencia, k):
    """Bins contiguos alrededor de k con potencia >= un cuarto del pico (-6 dB)."""
    limite = potencia[k] / 4
    izq = k
    while izq > 0 and potencia[izq - 1] >= limite:
        izq -= 1
    der = k
    while der < len(potencia) - 1 and potencia[der + 1] >= limite:
        der += 1
    return der - izq + 1


def candidatos_bloque(bloque, fs=FS, umbral_db=UMBRAL_DB, ancho_max=ANCHO_MAX_BINS):
    """Aplica los criterios instantáneos a un bloque.

    Devuelve (aceptados, rechazados). Cada elemento es un dict con la frecuencia
    refinada, la altura sobre el piso y, para rechazados, el criterio que falló.
    """
    n = len(bloque)
    p = espectro_potencia(bloque)
    piso = piso_ruido(p)
    # El 1e-30 evita log(0) en bins exactamente nulos (señales sintéticas).
    p_db = 10 * np.log10(np.maximum(p, 1e-30))
    piso_db = 10 * np.log10(np.maximum(piso, 1e-30))
    k_max = n // 2 - MARGEN_NYQUIST

    aceptados, rechazados = [], []
    # Máximo local estricto a la izquierda: evita contar dos veces una meseta.
    maximos = np.flatnonzero((p[1:-1] > p[:-2]) & (p[1:-1] >= p[2:])) + 1
    for k in maximos:
        snr_local = p_db[k] - piso_db[k]
        if snr_local < umbral_db:
            continue  # C1: la gran mayoría de máximos son ruido; no se registran.
        info = {'k': int(k), 'snr_local_db': float(snr_local)}
        if not K_MIN <= k <= k_max:
            info['falla'] = 'C2 fuera de rango (DC o Nyquist)'
            rechazados.append(info)
            continue
        ancho = ancho_6db(p, k)
        info['ancho_bins'] = ancho
        if ancho > ancho_max:
            info['falla'] = 'C3 demasiado ancho (no es un tono)'
            rechazados.append(info)
            continue
        d, pico_db = interpolar_pico(p_db, k)
        info['f_hz'] = float((k + d) * fs / n)
        # Amplitud del seno: |X| = A * suma(w) / 2 y suma(w) = n/2 para Hann.
        info['amplitud'] = float(4 * 10 ** (pico_db / 20) / n)
        aceptados.append(info)
    return aceptados, rechazados


class Receptor:
    """Seguimiento entre bloques: persistencia (C4), histéresis y clasificación."""

    def __init__(self, fs=FS, n=N_BLOQUE, banda_util=BANDA_UTIL):
        self.fs, self.n, self.banda_util = fs, n, banda_util
        self.pistas = []      # una pista por frecuencia observada
        self.secuencia = 0    # número de bloque, para detectar pérdidas en el destino

    def procesar(self, bloque):
        """Procesa un bloque y devuelve la lista de frecuencias validadas."""
        if len(bloque) != self.n:
            raise ValueError(f'Se esperaban {self.n} muestras.')
        candidatos, _ = candidatos_bloque(bloque, self.fs)
        tol_hz = TOLERANCIA_BINS * self.fs / self.n
        usadas = set()
        for pista in self.pistas:
            # Se asocia el candidato libre más cercano; si nadie cae dentro de la
            # tolerancia, la pista registra una ausencia en este bloque.
            libres = [i for i in range(len(candidatos)) if i not in usadas]
            mejor = min(libres, key=lambda i: abs(candidatos[i]['f_hz'] - pista['f_hz']),
                        default=None)
            if mejor is not None and abs(candidatos[mejor]['f_hz'] - pista['f_hz']) <= tol_hz:
                usadas.add(mejor)
                c = candidatos[mejor]
                # Promedio exponencial: suaviza la variación de la interpolación entre bloques.
                pista['f_hz'] = 0.7 * pista['f_hz'] + 0.3 * c['f_hz']
                pista['amplitud'] = c['amplitud']
                pista['snr_local_db'] = c['snr_local_db']
                pista['historia'].append(True)
            else:
                pista['historia'].append(False)
        for i, c in enumerate(candidatos):
            if i not in usadas:
                historia = deque([False] * (M_HISTORIA - 1) + [True], maxlen=M_HISTORIA)
                self.pistas.append({**c, 'historia': historia, 'validada': False})

        for pista in self.pistas:
            presentes = sum(pista['historia'])
            # Dos umbrales distintos evitan que la muesca del nodo espectral
            # se encienda y apague en cada bloque (se escucharía como chasquido).
            if presentes >= K_ACEPTAR:
                pista['validada'] = True
            elif presentes <= K_LIBERAR:
                pista['validada'] = False
        self.pistas = [p for p in self.pistas if any(p['historia'])]
        self.secuencia = (self.secuencia + 1) % 256
        return self.clasificar([p for p in self.pistas if p['validada']])

    def clasificar(self, validadas):
        """Marca cada frecuencia como útil, interferente y/o armónica (C5)."""
        lo, hi = self.banda_util
        df = self.fs / self.n
        salida = []
        for p in validadas:
            flags = 0
            if lo <= p['f_hz'] <= hi:
                flags |= FLAG_BANDA_UTIL  # se protege: no se envía como ruido a eliminar
            else:
                flags |= FLAG_INTERFERENTE
            salida.append({'f_hz': p['f_hz'], 'amplitud': p['amplitud'],
                           'snr_local_db': p['snr_local_db'], 'flags': flags})
        interferentes = sorted((s for s in salida if s['flags'] & FLAG_INTERFERENTE),
                               key=lambda s: s['f_hz'])
        if interferentes:
            f0 = interferentes[0]['f_hz']
            for s in interferentes[1:]:
                h = round(s['f_hz'] / f0)
                # Armónico si está a menos de un bin de un múltiplo entero de f0 (50/60 Hz).
                if h >= 2 and abs(s['f_hz'] - h * f0) <= df:
                    s['flags'] |= FLAG_ARMONICO
        # La trama tiene espacio limitado: primero las interferentes (son las que
        # los nodos 2 y 3 deben atacar) y, dentro de cada grupo, las más fuertes.
        salida.sort(key=lambda s: (not s['flags'] & FLAG_INTERFERENTE, -s['amplitud']))
        return salida[:MAX_REPORTADAS]


def validar_contra_emisor(validadas, tabla=FRECUENCIAS_EMISOR,
                          tolerancia=TOLERANCIA_EMISOR_HZ, rango_db=RANGO_RELATIVO_DB):
    """C6: compara las frecuencias validadas con las que el emisor debía enviar.

    Devuelve (reporte, no_esperadas). El reporte trae, por cada frecuencia de la
    tabla, si se recibió, el error en Hz y la amplitud; no_esperadas son las
    validadas que no corresponden a ninguna frecuencia del emisor y que están a
    menos de `rango_db` del tono más fuerte.
    """
    reporte, usadas = [], set()
    for f_tabla in tabla:
        cerca = [i for i, v in enumerate(validadas)
                 if i not in usadas and abs(v['f_hz'] - f_tabla) <= tolerancia]
        if cerca:
            # Si dos validadas caen en la tolerancia se toma la más cercana.
            i = min(cerca, key=lambda j: abs(validadas[j]['f_hz'] - f_tabla))
            usadas.add(i)
            v = validadas[i]
            reporte.append({'f_emisor_hz': f_tabla, 'recibida': True,
                            'error_hz': v['f_hz'] - f_tabla, 'amplitud': v['amplitud']})
        else:
            reporte.append({'f_emisor_hz': f_tabla, 'recibida': False,
                            'error_hz': None, 'amplitud': None})
    # El tono más fuerte fija la referencia; lo que queda 40 dB por debajo se ignora.
    piso = max((v['amplitud'] for v in validadas), default=0) * 10 ** (-rango_db / 20)
    no_esperadas = [v for i, v in enumerate(validadas)
                    if i not in usadas and v['amplitud'] >= piso]
    return reporte, no_esperadas


def crc16_ccitt(datos, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (polinomio 0x1021), el mismo que usa full_system/sistema_ruido/link.py."""
    for byte in datos:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def empaquetar(secuencia, frecuencias):
    """Trama de parámetros nodo 1 -> nodos 2 y 3.

    | 0xA5 0x5A | tipo 0x02 | sec | n | n x (f*4 u16 LE, snr i8, flags u8) | CRC16 LE |
    5 + 4n + 2 bytes; con n = 8 son 39 bytes cada 32 ms.
    """
    cuerpo = bytearray([0x02, secuencia & 0xFF, len(frecuencias)])
    for s in frecuencias:
        # Cuartos de Hz: resolución 0.25 Hz y hasta 16383 Hz en 16 bits.
        f = int(round(s['f_hz'] * 4))
        snr = int(np.clip(round(s['snr_local_db']), -128, 127))
        cuerpo += f.to_bytes(2, 'little') + snr.to_bytes(1, 'little', signed=True)
        cuerpo.append(s['flags'])
    trama = bytearray([0xA5, 0x5A]) + cuerpo
    return bytes(trama + crc16_ccitt(trama).to_bytes(2, 'little'))


def desempaquetar(trama):
    """Lado receptor de la trama: verifica sincronía, longitud y CRC."""
    if len(trama) < 7 or trama[0] != 0xA5 or trama[1] != 0x5A or trama[2] != 0x02:
        raise ValueError('Trama sin encabezado válido.')
    n = trama[4]
    if len(trama) != 7 + 4 * n:
        raise ValueError('Longitud inconsistente.')
    if crc16_ccitt(trama[:-2]) != int.from_bytes(trama[-2:], 'little'):
        raise ValueError('CRC incorrecto: se descarta el bloque.')
    salida = []
    for i in range(n):
        base = 5 + 4 * i
        salida.append({'f_hz': int.from_bytes(trama[base:base + 2], 'little') / 4,
                       'snr_local_db': int.from_bytes(trama[base + 2:base + 3], 'little',
                                                      signed=True),
                       'flags': trama[base + 3]})
    return trama[3], salida


def procesar_senal(x, receptor=None):
    """Recorre una señal completa en bloques de N con salto N/2 (como el nodo 1)."""
    receptor = receptor or Receptor()
    resultados = []
    for inicio in range(0, len(x) - receptor.n + 1, SALTO):
        resultados.append(receptor.procesar(x[inicio:inicio + receptor.n]))
    return resultados