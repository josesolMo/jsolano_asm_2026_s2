"""Regenera resultados exclusivos del filtrado temporal sin tocar los originales."""

import csv
import hashlib
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.io import wavfile

from .filtros_temporales import ATOL, RTOL, FS, RETARDO, disenar_filtro, filtrar_manual, medir_respuesta


def leer_pcm16(ruta):
    """Conserva la escala del generador y rechaza formatos incompatibles."""
    # La comprobación impide evaluar inadvertidamente otra frecuencia o formato.
    fs, datos = wavfile.read(ruta)
    if fs != FS or datos.dtype != np.int16 or datos.ndim != 1 or len(datos) != 96000:
        raise ValueError(f'WAV incompatible: {ruta}')
    return datos.astype(np.float64) / 32767


def validar(entrada, coeficientes, nombre, bloque=1024):
    """Compara sin alineación: las dos implementaciones son causales."""
    # Las mismas condiciones nulas permiten comparar muestra a muestra.
    manual, _ = filtrar_manual(entrada, coeficientes)
    referencia = signal.lfilter(coeficientes, [1.0], entrada)
    estado = None
    partes = []
    for inicio in range(0, len(entrada), bloque):
        parte, estado = filtrar_manual(entrada[inicio:inicio + bloque], coeficientes, estado)
        partes.append(parte)
    por_bloques = np.concatenate(partes)
    np.testing.assert_allclose(manual, referencia, atol=ATOL, rtol=RTOL)
    np.testing.assert_allclose(manual, por_bloques, atol=ATOL, rtol=RTOL)
    return manual, dict(caso=nombre, atol=ATOL, rtol=RTOL,
                        error_max_scipy=float(np.max(np.abs(manual - referencia))),
                        error_max_bloques=float(np.max(np.abs(manual - por_bloques))),
                        bloque_muestras=bloque, ultimo_bloque_muestras=len(entrada) % bloque)


def guardar_csv(ruta, filas):
    # Un esquema explícito conserva las mediciones en archivos independientes.
    with ruta.open('w', newline='', encoding='utf-8') as archivo:
        escritor = csv.DictWriter(archivo, fieldnames=list(filas[0]))
        escritor.writeheader()
        escritor.writerows(filas)


def guardar_figura(figura, ruta):
    # Cerrar las figuras libera memoria cuando el notebook regenera todo.
    figura.tight_layout()
    figura.savefig(ruta, dpi=150)
    plt.close(figura)


def caracterizar(coeficientes, carpeta):
    # La rejilla incluye Nyquist y permite ver paso, transición y rechazo.
    f = np.linspace(0, FS / 2, 65537)
    _, h = signal.freqz(coeficientes, [1.0], worN=f, fs=FS)
    db = 20 * np.log10(np.maximum(np.abs(h), 1e-15))
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(f, db, label='FIR float64')
    ax.axvspan(0, 1500, alpha=.12, color='green', label='Paso')
    ax.axvspan(1500, 2200, alpha=.12, color='orange', label='Transición')
    ax.axvspan(2200, 8000, alpha=.08, color='red', label='Rechazo')
    delta = (10**(.5 / 20) - 1) / (10**(.5 / 20) + 1)
    ax.hlines([20*np.log10(1-delta), 20*np.log10(1+delta)], 0, 1500,
              color='green', linestyle='--', label='Límites de paso (0.5 dB pico a pico)')
    ax.hlines(-40, 2200, 8000, color='red', linestyle='--', label='Rechazo mínimo 40 dB')
    ax.set(title='Magnitud del FIR temporal', xlabel='Frecuencia (Hz)', ylabel='Ganancia (dB)', ylim=(-100, 5))
    ax.legend(fontsize=8)
    guardar_figura(fig, carpeta / 'temporal_magnitud.png')
    # Se ocultan regiones bajo -60 dB; no se interpreta fase en ceros.
    visible = np.abs(h) >= 1e-3
    fase = np.unwrap(np.angle(h))
    fase[~visible] = np.nan
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(f, fase, label='Fase; |H| >= -60 dB')
    ax.set(title='Fase del FIR causal', xlabel='Frecuencia (Hz)', ylabel='Fase (rad)')
    ax.legend()
    guardar_figura(fig, carpeta / 'temporal_fase.png')
    # La derivada logarítmica evita evaluar el retardo en denominadores casi nulos.
    _, ponderada = signal.freqz(np.arange(len(coeficientes))*coeficientes, [1.0], worN=f, fs=FS)
    retardo = np.full(len(f), np.nan)
    retardo[visible] = np.real(ponderada[visible] / h[visible])
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(f, retardo, label='Retardo medido; |H| >= -60 dB')
    ax.axhline(RETARDO, color='red', linestyle='--', label='20 muestras = 1.25 ms')
    ax.set(title='Retardo de grupo del FIR', xlabel='Frecuencia (Hz)', ylabel='Retardo (muestras)', ylim=(19, 21))
    ax.legend()
    guardar_figura(fig, carpeta / 'temporal_retardo.png')
    # H(z)=B(z)/z^40 tiene 40 polos en el origen aunque a=[1.0].
    ceros = np.roots(coeficientes)
    fig, ax = plt.subplots(figsize=(6, 6))
    angulo = np.linspace(0, 2*np.pi, 500)
    ax.plot(np.cos(angulo), np.sin(angulo), '--', color='gray', label='Círculo unitario')
    ax.scatter(ceros.real, ceros.imag, facecolors='none', edgecolors='blue', label='40 ceros')
    ax.scatter([0], [0], marker='x', color='red', label='40 polos en el origen')
    ax.set(title='Polos y ceros de B(z)/z^40', xlabel='Parte real (adimensional)', ylabel='Parte imaginaria (adimensional)')
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=8)
    guardar_figura(fig, carpeta / 'temporal_polos_ceros.png')
    return float(np.max(np.abs(retardo[visible] - RETARDO)))


def ejecutar_analisis(raiz):
    """Verifica los datos, aplica un único filtro y guarda sus entregables."""
    # Todas las rutas derivan de la raíz recibida para admitir ejecución aislada.
    raiz = Path(raiz)
    resultados = raiz / 'resultados'
    figuras = resultados / 'figuras'
    figuras.mkdir(parents=True, exist_ok=True)
    configuracion = json.loads((raiz / 'metadatos/configuraciones.json').read_text(encoding='utf-8'))
    b = disenar_filtro()
    respuesta = medir_respuesta(b)
    respuesta32 = medir_respuesta(b.astype(np.float32))
    for medicion in (respuesta, respuesta32):
        assert medicion['rizado_dB'] <= .5 and medicion['atenuacion_rechazo_dB'] >= 40
    np.testing.assert_allclose(b, b[::-1], atol=1e-15, rtol=0)
    error_retardo = caracterizar(b, figuras)
    # El impulso también comprueba el orden de los coeficientes y la cola nula.
    impulso = np.zeros(97)
    impulso[0] = 1
    salida_impulso, registro = validar(impulso, b, 'impulso', bloque=17)
    np.testing.assert_allclose(salida_impulso[:41], b, atol=ATOL, rtol=RTOL)
    np.testing.assert_allclose(salida_impulso[41:], 0, atol=ATOL, rtol=RTOL)
    validaciones = [registro]
    metricas = []
    datos_verificados = []
    for nombre, cfg in configuracion['casos'].items():
        # Los hashes prueban que los nueve WAV corresponden a los metadatos.
        assert cfg['fs_Hz'] == FS and cfg['N'] == 96000 and cfg['duracion_s'] == 6
        assert cfg['frecuencias_utiles_Hz'] == [440, 880, 1320]
        esperado = {'caso_1': ('tonal', 5), 'caso_2': ('blanco', 3), 'caso_3': ('banda', 0)}[nombre]
        assert (cfg['tipo_ruido'], cfg['snr_objetivo_dB']) == esperado
        if nombre == 'caso_1':
            assert cfg['frecuencia_interferencia_Hz'] == 2500
        if nombre == 'caso_3':
            assert cfg['banda_ruido_Hz'] == [1000, 2200]
        senales = {}
        for archivo, info in cfg['archivos'].items():
            ruta = raiz / info['ruta']
            assert hashlib.sha256(ruta.read_bytes()).hexdigest() == info['sha256']
            senales[archivo] = leer_pcm16(ruta)
        limpia = senales['01_referencia_limpia.wav']
        entrada = senales['03_entrada_contaminada.wav']
        salida, registro = validar(entrada, b, nombre)
        validaciones.append(registro)
        snr_total = float(10*np.log10(np.mean(limpia**2)/np.mean((entrada-limpia)**2)))
        assert abs(snr_total - cfg['snr_objetivo_dB']) <= .01
        datos_verificados.append(dict(caso=nombre, fs_Hz=FS, N=len(entrada), duracion_s=6,
                                     snr_total_dB=snr_total, hashes_correctos=True))
        # Se elimina el arranque causal: y[40:N] corresponde a limpia[20:N-20].
        # No hay cola añadida ni corrección de amplitud o fase de los tonos.
        referencia = limpia[20:-20]
        contaminada = entrada[20:-20]
        filtrada = salida[40:]
        potencia = np.mean(referencia**2)
        mse = float(np.mean((filtrada-referencia)**2))
        snr_entrada = float(10*np.log10(potencia/np.mean((contaminada-referencia)**2)))
        snr_salida = float(10*np.log10(potencia/mse))
        pico = float(np.max(np.abs(salida)))
        cota = float(np.max(np.abs(entrada))*np.sum(np.abs(b)))
        metricas.append(dict(caso=nombre, snr_entrada_dB=snr_entrada, mse_salida=mse,
                             snr_salida_dB=snr_salida, delta_snr_dB=snr_salida-snr_entrada,
                             referencia_inicio=20, referencia_fin_exclusivo=len(entrada)-20,
                             salida_inicio=40, salida_fin_exclusivo=len(salida),
                             muestras_evaluadas=len(referencia), pico_salida=pico,
                             margen_amplitud=1-pico, cota_amplitud=cota,
                             muestras_fuera_rango=int(np.sum(np.abs(salida)>1))))
        # La salida float64 es autoritativa; el WAV PCM16 es solo una copia cuantizada.
        destino = resultados / 'filtrado_temporal' / nombre
        destino.mkdir(parents=True, exist_ok=True)
        np.save(destino / 'salida_float64.npy', salida)
        if pico > 1:
            raise ValueError('La salida supera el rango PCM16; revisar antes de exportar.')
        wavfile.write(destino / '04_filtrada_temporal.wav', FS, np.int16(salida*32767))
        # Tiempo y espectro usan exactamente el mismo intervalo alineado.
        fig, axes = plt.subplots(2, 1, figsize=(11, 7))
        tiempo = np.arange(20, len(entrada)-20)/FS
        ventana = slice(1600, 2080)
        for vector, etiqueta in [(referencia, 'Limpia'), (contaminada, 'Contaminada'), (filtrada, 'Filtrada alineada')]:
            axes[0].plot(tiempo[ventana]*1000, vector[ventana], label=etiqueta, linewidth=.9)
            hann = signal.windows.hann(len(vector), sym=False)
            espectro = np.abs(np.fft.rfft(vector*hann))/(hann.sum()/2)
            frecuencias = np.fft.rfftfreq(len(vector), 1/FS)
            axes[1].plot(frecuencias, 20*np.log10(np.maximum(espectro, 1e-12)), label=etiqueta, linewidth=.8)
        axes[0].set(title=f'{nombre}: comparación temporal alineada', xlabel='Tiempo de referencia (ms)', ylabel='Amplitud (escala PCM16)')
        axes[1].set(title=f'{nombre}: espectro de amplitud con ventana Hann', xlabel='Frecuencia (Hz)', ylabel='Amplitud (dB respecto a 1)', ylim=(-120, 0))
        for ax in axes:
            ax.legend()
        guardar_figura(fig, figuras / f'temporal_{nombre}.png')
    # Especificaciones y cuantización quedan separadas de configuraciones.json.
    metadatos = dict(fs_Hz=FS, orden=40, coeficientes=41, b=b.tolist(), a=[1.0],
                     bandas_Hz=[0, 1500, 2200, 8000], ganancias=[1, 0],
                     rizado_max_dB=.5, rechazo_min_dB=40, metodo='scipy.signal.remez',
                     delta_p=(10**(.5/20)-1)/(10**(.5/20)+1), delta_s=.01,
                     retardo_muestras=20, retardo_ms=1.25, respuesta_float64=respuesta,
                     respuesta_coeficientes_float32=respuesta32,
                     error_max_coeficientes_float32=float(np.max(np.abs(b-b.astype(np.float32)))),
                     error_max_retardo_muestras=error_retardo, suma_absoluta_coeficientes=float(np.sum(np.abs(b))),
                     coeficientes_float32_bytes=164, estado_float32_bytes=160,
                     buffer_circular_alternativo_bytes=164, multiplicaciones_por_segundo=656000,
                     datos_verificados=datos_verificados)
    (raiz / 'metadatos/filtro_temporal.json').write_text(json.dumps(metadatos, indent=2, ensure_ascii=False), encoding='utf-8')
    guardar_csv(resultados / 'metricas_temporales.csv', metricas)
    guardar_csv(resultados / 'validacion_temporal.csv', validaciones)
    return metadatos, metricas, validaciones


if __name__ == '__main__':
    # Permite regenerar fuera de Jupyter desde la raíz del repositorio.
    informacion, mediciones, comprobaciones = ejecutar_analisis(Path(__file__).resolve().parents[1])
    print(json.dumps(dict(respuesta=informacion['respuesta_float64'], metricas=mediciones,
                          validaciones=comprobaciones), indent=2, ensure_ascii=False))
