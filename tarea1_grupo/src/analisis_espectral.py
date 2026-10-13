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


def atenuacion_db(entrada, salida, frecuencia):
    """Atenuación de un tono: potencia en su bin antes y después, sobre toda la señal."""
    # Una FFT larga con Hann da resolución fina (16000/96000 ≈ 0.17 Hz) y poca fuga.
    ventana = np.hanning(len(entrada))
    k = int(round(frecuencia * len(entrada) / FS))
    zona = slice(k - 3, k + 4)  # Margen por si la frecuencia estimada cae entre bins.
    p_entrada = np.max(np.abs(np.fft.rfft(entrada * ventana)[zona]) ** 2)
    p_salida = np.max(np.abs(np.fft.rfft(salida * ventana)[zona]) ** 2)
    return float(10 * np.log10(p_entrada / p_salida))


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


TONOS_HZ = (440.0, 880.0, 1320.0)  # De metadatos/configuraciones.json.
ALFAS_BARRIDO = (0.25, 0.5, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 64)
COLORES_TONOS = ('#2a78d6', '#eb6834', '#1baf7a')  # Orden fijo: un color por tono.


def energia_tonos(senal, tonos=TONOS_HZ):
    """Energía de cada tono útil, medida en su bin de una FFT de toda la señal."""
    # Con 96000 muestras la resolución es ~0.17 Hz: cada tono cae en un bin exacto y
    # el ruido que comparte ese bin es despreciable frente al tono.
    espectro = np.abs(np.fft.rfft(senal * np.hanning(len(senal)))) ** 2
    bins = [int(round(f * len(senal) / FS)) for f in tonos]
    # ±2 bins recogen el lóbulo principal de la ventana Hann.
    return np.array([espectro[k - 2:k + 3].sum() for k in bins])


def barrido_alfa(raiz=RAIZ, casos=(2, 3), alfas=ALFAS_BARRIDO):
    """Varía alfa y mide SNR y energía conservada de cada tono (inciso 2.4f)."""
    filas = []
    for caso in casos:
        carpeta = raiz / 'pruebas' / f'caso_{caso}'
        limpia = leer_wav(carpeta / '01_referencia_limpia.wav')
        entrada = leer_wav(carpeta / '03_entrada_contaminada.wav')
        referencia = energia_tonos(limpia)
        for alfa in alfas:
            salida = reducir_ruido(entrada, alfa=alfa)
            conservada = 100 * energia_tonos(salida) / referencia
            fila = dict(caso=caso, alfa=alfa, snr_salida_dB=round(snr_db(limpia, salida), 3))
            fila.update({f'energia_{int(f)}Hz_pct': round(p, 2) for f, p in zip(TONOS_HZ, conservada)})
            filas.append(fila)

    with open(raiz / 'resultados/barrido_alfa.csv', 'w', newline='', encoding='utf-8') as archivo:
        escritor = csv.DictWriter(archivo, fieldnames=list(filas[0]))
        escritor.writeheader()
        escritor.writerows(filas)
    figura_compromiso(filas, casos, raiz / 'resultados/figuras/espectral_compromiso.png')
    return filas


def figura_compromiso(filas, casos, ruta):
    """Arriba la SNR y abajo la energía de cada tono, ambas contra alfa (escala log)."""
    # Dos filas en vez de doble eje y: SNR (dB) y energía (%) tienen escalas distintas.
    figura, ejes = plt.subplots(2, len(casos), figsize=(10, 6.5), sharex=True, sharey='row')
    for col, caso in enumerate(casos):
        datos = [f for f in filas if f['caso'] == caso]
        alfas = [f['alfa'] for f in datos]
        ax_snr, ax_e = ejes[0, col], ejes[1, col]

        ax_snr.plot(alfas, [f['snr_salida_dB'] for f in datos], c='0.25', lw=2, marker='o', ms=4)
        ax_snr.set_title(f'Caso {caso}')
        for f, color in zip(TONOS_HZ, COLORES_TONOS):
            ax_e.plot(alfas, [d[f'energia_{int(f)}Hz_pct'] for d in datos], c=color, lw=2,
                      marker='o', ms=4, label=f'{int(f)} Hz')
        for ax in (ax_snr, ax_e):
            # Línea punteada en el alfa elegido para ubicar el punto de operación.
            ax.axvline(ALFA, c='0.6', ls=':', lw=1)
            ax.grid(alpha=.3)
        ax_e.set(xscale='log', xlabel='α (factor de sobresustracción)', ylim=(-5, 105))
        ax_e.set_xticks([0.25, 1, 4, 16, 64], ['0.25', '1', '4', '16', '64'])
    ejes[0, 0].set_ylabel('SNR de salida (dB)')
    ejes[1, 0].set_ylabel('Energía conservada (%)')
    ejes[1, 0].legend(title='Tono útil', fontsize=8, loc='lower left')
    ejes[0, -1].annotate(f'α = {ALFA:g} (elegido)', (ALFA, ejes[0, -1].get_ylim()[0]),
                         xytext=(4, 6), textcoords='offset points', fontsize=8, color='0.4')
    figura.tight_layout()
    figura.savefig(ruta, dpi=120)
    plt.close(figura)


def ejecutar_analisis(raiz=RAIZ):
    """Procesa los tres casos y devuelve la tabla de métricas."""
    filas = []
    for caso in CASOS:
        carpeta = raiz / 'pruebas' / f'caso_{caso}'
        limpia = leer_wav(carpeta / '01_referencia_limpia.wav')
        entrada = leer_wav(carpeta / '03_entrada_contaminada.wav')
        registro = []
        salida = reducir_ruido(entrada, registro=registro)
        detectadas = np.concatenate(registro)
        # La mediana resume la frecuencia estimada en todos los bloques que detectaron algo.
        f_interferente = float(np.median(detectadas)) if len(detectadas) else None

        guardar_wav(raiz / f'resultados/filtrado_espectral/caso_{caso}/04_filtrada_espectral.wav', salida)
        figura_caso(caso, limpia, entrada, salida, raiz / f'resultados/figuras/espectral_caso_{caso}.png')

        snr_entrada, snr_salida = snr_db(limpia, entrada), snr_db(limpia, salida)
        filas.append(dict(caso=caso, N=N_BLOQUE, alfa=ALFA, g_min=G_MIN, ancho_mediana=ANCHO_MEDIANA,
                          snr_entrada_dB=round(snr_entrada, 3), snr_salida_dB=round(snr_salida, 3),
                          mejora_snr_dB=round(snr_salida - snr_entrada, 3),
                          mse_entrada=float(np.mean((limpia - entrada) ** 2)),
                          mse_salida=float(np.mean((limpia - salida) ** 2)),
                          bloques_con_pico_pct=round(100 * np.mean([len(r) > 0 for r in registro]), 1),
                          frecuencia_interferente_Hz=f_interferente,
                          atenuacion_interferencia_dB=(round(atenuacion_db(entrada, salida, f_interferente), 2)
                                                       if f_interferente else None)))

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
        if fila['frecuencia_interferente_Hz']:
            print(f"  interferencia estimada en {fila['frecuencia_interferente_Hz']:.1f} Hz "
                  f"({fila['bloques_con_pico_pct']}% de bloques), "
                  f"atenuada {fila['atenuacion_interferencia_dB']:.1f} dB")
    print('Barrido de alfa (2.4f)...')
    for fila in barrido_alfa():
        if fila['alfa'] in (1, ALFA, 64):
            print(f"  Caso {fila['caso']}, α={fila['alfa']:g}: SNR {fila['snr_salida_dB']:.1f} dB, "
                  f"energía 440/880/1320 Hz = {fila['energia_440Hz_pct']:.0f}/"
                  f"{fila['energia_880Hz_pct']:.0f}/{fila['energia_1320Hz_pct']:.0f} %")
    print('Resultados guardados en tarea1_grupo/resultados/')