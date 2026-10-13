"""Aplica la reducción espectral a los tres casos y guarda audio, figuras y métricas.

Uso, desde la raíz del repositorio:
    python -m tarea1_grupo.src.analisis_espectral
"""

import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # Genera los PNG sin abrir ventanas (funciona también sin pantalla).
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile

from .filtrado_espectral import (
    ALFA, ANCHO_MEDIANA, FS, G_MIN, N_BLOQUE, leer_wav, reducir_ruido, snr_db)

RAIZ = Path(__file__).resolve().parents[1]
CASOS = (1, 2, 3)


def guardar_wav(ruta, senal):
    """Guarda en PCM16 con la misma escala que leer_wav."""
    # Recortar a [-1, 1] evita que un valor fuera de rango se desborde al pasar a int16.
    ruta.parent.mkdir(parents=True, exist_ok=True)
    wavfile.write(ruta, FS, np.round(np.clip(senal, -1, 1) * 32767).astype(np.int16))


def espectro_db(senal, inicio, n=N_BLOQUE):
    """Magnitud en dB de un bloque con ventana Hann, solo para graficar."""
    ventana = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(n) / n)
    # El 1e-9 evita log(0) en bins vacíos.
    return 20 * np.log10(np.abs(np.fft.rfft(senal[inicio:inicio + n] * ventana)) + 1e-9)


def figura_caso(caso, limpia, entrada, salida, ruta):
    """Espectro de un bloque y fragmento temporal: entrada, salida y referencia."""
    inicio = 3 * N_BLOQUE  # Un bloque alejado del inicio, ya en régimen estable.
    frecuencias = np.fft.rfftfreq(N_BLOQUE, 1 / FS)
    figura, (ax_f, ax_t) = plt.subplots(2, 1, figsize=(9, 6.5))

    ax_f.plot(frecuencias, espectro_db(entrada, inicio), c='0.7', lw=1, label='entrada contaminada')
    ax_f.plot(frecuencias, espectro_db(salida, inicio), c='C0', lw=1, label='salida espectral')
    ax_f.plot(frecuencias, espectro_db(limpia, inicio), c='C2', lw=1, ls='--', label='referencia limpia')
    ax_f.set(xlim=(0, 4000), ylim=(-120, 60), xlabel='Frecuencia (Hz)', ylabel='Magnitud (dB)',
             title=f'Caso {caso}: SNR {snr_db(limpia, entrada):.1f} dB -> {snr_db(limpia, salida):.1f} dB')
    ax_f.grid(alpha=.3)
    ax_f.legend(fontsize=8, loc='upper right')

    # 20 ms bastan para ver varios periodos de 440 Hz.
    tramo = slice(inicio, inicio + int(0.02 * FS))
    t_ms = np.arange(tramo.stop - tramo.start) / FS * 1000
    ax_t.plot(t_ms, entrada[tramo], c='0.7', lw=1, label='entrada contaminada')
    ax_t.plot(t_ms, salida[tramo], c='C0', lw=1.2, label='salida espectral')
    ax_t.plot(t_ms, limpia[tramo], c='C2', lw=1, ls='--', label='referencia limpia')
    ax_t.set(xlabel='Tiempo (ms)', ylabel='Amplitud')
    ax_t.grid(alpha=.3)

    figura.tight_layout()
    ruta.parent.mkdir(parents=True, exist_ok=True)
    figura.savefig(ruta, dpi=120)
    plt.close(figura)


def ejecutar_analisis(raiz=RAIZ):
    """Procesa los tres casos y devuelve la tabla de métricas."""
    filas = []
    for caso in CASOS:
        carpeta = raiz / 'pruebas' / f'caso_{caso}'
        limpia = leer_wav(carpeta / '01_referencia_limpia.wav')
        entrada = leer_wav(carpeta / '03_entrada_contaminada.wav')
        salida = reducir_ruido(entrada)

        guardar_wav(raiz / f'resultados/filtrado_espectral/caso_{caso}/04_filtrada_espectral.wav', salida)
        figura_caso(caso, limpia, entrada, salida, raiz / f'resultados/figuras/espectral_caso_{caso}.png')

        snr_entrada, snr_salida = snr_db(limpia, entrada), snr_db(limpia, salida)
        filas.append(dict(caso=caso, N=N_BLOQUE, alfa=ALFA, g_min=G_MIN, ancho_mediana=ANCHO_MEDIANA,
                          snr_entrada_dB=round(snr_entrada, 3), snr_salida_dB=round(snr_salida, 3),
                          mejora_snr_dB=round(snr_salida - snr_entrada, 3),
                          mse_entrada=float(np.mean((limpia - entrada) ** 2)),
                          mse_salida=float(np.mean((limpia - salida) ** 2))))

    ruta_csv = raiz / 'resultados/metricas_espectrales.csv'
    ruta_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(ruta_csv, 'w', newline='', encoding='utf-8') as archivo:
        escritor = csv.DictWriter(archivo, fieldnames=list(filas[0]))
        escritor.writeheader()
        escritor.writerows(filas)
    return filas


if __name__ == '__main__':
    for fila in ejecutar_analisis():
        print(f"Caso {fila['caso']}: SNR {fila['snr_entrada_dB']:.2f} dB -> "
              f"{fila['snr_salida_dB']:.2f} dB (mejora {fila['mejora_snr_dB']:+.2f} dB)")
    print('Resultados guardados en tarea1_grupo/resultados/')