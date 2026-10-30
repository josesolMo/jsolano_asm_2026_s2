"""Parametros globales y directorio de salida del sistema."""
from pathlib import Path

DIR_SALIDA = Path("taller-fft-sys/resultados_sistema")
DIR_SALIDA.mkdir(exist_ok=True)


FS = 16_000            # Hz. Nyquist = 8 kHz; mismo valor que la Tarea 1.
N = 1024               # muestras por bloque = tamano de FFT (potencia de 2). 64 ms.
HOP = N // 2           # avance del nodo espectral (50 % de solape)
N_BLOQUES = 64         # duracion de cada prueba: 64 bloques = 65 536 muestras = 4.096 s
L = N * N_BLOQUES
BIN_HZ = FS / N        # resolucion de la FFT: 15.625 Hz
ADC_BITS = 12          # resolucion del ADC (y del DAC) del microcontrolador
DAC_BITS = 12
PICO = 0.8             # amplitud maxima de las senales (deja margen al ADC)
FADE_MS = 20           # rampa de entrada/salida de la senal limpia
F_RED = 60             # Hz, red electrica de Costa Rica
TRANS_HZ = 700         # ancho de transicion entre banda de paso y de rechazo
RIZADO_DB = 0.5        # rizado maximo en banda de paso: 20log10(max|H|/min|H|)
ATEN_DB = 40           # atenuacion minima en banda de rechazo
LAMBDA_EMA = 0.9       # memoria del promedio exponencial del espectro
K_MEDIANA = 51         # bins del filtro de mediana que estima el piso de ruido (~800 Hz)
UMBRAL_PICO_DB = 15.0  # un pico debe superar el piso en esta cantidad
RANGO_PICO_DB = 40.0   # y estar a menos de esto del pico mas fuerte
GUARDA_BINS = 8        # margen alrededor de la banda util antes de llamar interferente
NOTCH_BINS = 4         # semiancho (bins) del rechazo de cada interferente detectado
ALFA = 3.0             # sobre-estimacion del ruido (parametro adaptativo)
BETA = 0.1             # ganancia minima (-20 dB) para no dejar huecos totales
G_INTERF = 0.01        # ganancia en los bins de un interferente detectado (-40 dB)
LAG_MAX = 64           # busqueda de retardo para alinear salida y referencia
