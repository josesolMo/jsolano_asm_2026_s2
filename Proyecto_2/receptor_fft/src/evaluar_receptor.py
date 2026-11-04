"""Mide los criterios del receptor con los casos de la Tarea 1 y pruebas sintéticas.

Ejecutar desde la raíz del repositorio:
    python -m receptor_fft.src.evaluar_receptor
Genera receptor_fft/resultados/*.csv y receptor_fft/resultados/figuras/*.png.
"""

import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, sosfilt

from receptor_fft.src.receptor_fft import (
    FLAG_ARMONICO, FLAG_INTERFERENTE, FRECUENCIAS_EMISOR, FS, N_BLOQUE, SALTO,
    TABLAS_POR_MODO, UMBRAL_DB, Receptor, candidatos_bloque, espectro_potencia,
    interpolar_pico, procesar_senal, validar_contra_emisor)

RAIZ = Path(__file__).resolve().parents[2]
SALIDA = RAIZ / 'receptor_fft' / 'resultados'
FIGURAS = SALIDA / 'figuras'
SEMILLA = 20261106
# Los WAV de la Tarea 1 pueden estar junto a esta carpeta o un nivel más arriba.
TAREA1 = next((p / 'tarea1_grupo' for p in (RAIZ, RAIZ.parent) if (p / 'tarea1_grupo').is_dir()),
              None)


def tres_tonos(t):
    """Señal útil de la Tarea 1: 440, 880 y 1320 Hz."""
    return (np.sin(2 * np.pi * 440 * t) + 0.55 * np.sin(2 * np.pi * 880 * t + 0.61)
            + 0.3 * np.sin(2 * np.pi * 1320 * t - 0.87)) * 0.3


def interferentes(resultado, f_ref=None, tol=2 * FS / N_BLOQUE):
    """Frecuencias marcadas como interferentes en un bloque (opcionalmente cerca de f_ref)."""
    fs_ = [s['f_hz'] for s in resultado if s['flags'] & FLAG_INTERFERENTE]
    if f_ref is None:
        return fs_
    return [f for f in fs_ if abs(f - f_ref) <= tol]


def fin_bloque_s(i):
    """Instante en que termina el bloque i: ahí el resultado ya está disponible."""
    return (i * SALTO + N_BLOQUE) / FS


def resumen(nombre, resultados, f_esperada, t_inicio=0.0, t_fin=None, tol=2 * FS / N_BLOQUE,
            tambien=()):
    """Fila de tabla: aciertos, falsas alarmas y latencia de una prueba.

    Los aciertos se cuentan sobre los bloques que caen por completo dentro de
    [t_inicio, t_fin]; las falsas alarmas son interferentes lejos de f_esperada
    y de las frecuencias de `tambien` (por ejemplo, armónicos que sí existen).
    """
    t_fin = t_fin if t_fin is not None else fin_bloque_s(len(resultados) - 1)
    dentro = [i for i in range(len(resultados))
              if fin_bloque_s(i) - N_BLOQUE / FS >= t_inicio and fin_bloque_s(i) <= t_fin]
    cerca = lambda r: interferentes(r, f_esperada, tol) if f_esperada else []
    aciertos = sum(bool(cerca(resultados[i])) for i in dentro)
    legitimas = [f_esperada, *tambien] if f_esperada else list(tambien)
    falsas = sum(1 for r in resultados for f in interferentes(r)
                 if all(abs(f - g) > tol for g in legitimas))
    primera = next((i for i, r in enumerate(resultados)
                    if fin_bloque_s(i) > t_inicio and cerca(r)), None)
    errores = [abs(f - f_esperada) for r in resultados for f in cerca(r)]
    return {
        'prueba': nombre, 'bloques': len(resultados),
        'f_esperada_Hz': f_esperada or '',
        'deteccion_pct': round(100 * aciertos / max(len(dentro), 1), 2) if f_esperada else '',
        'falsas_alarmas_total': falsas,
        'latencia_ms': round(1000 * (fin_bloque_s(primera) - t_inicio), 1)
        if primera is not None else '',
        'error_f_max_Hz': round(max(errores), 3) if errores else '',
    }


def pruebas_casos():
    """Casos 1-3 de la Tarea 1 tal como están en los WAV."""
    filas = []
    if TAREA1 is None:
        print('No se encontró tarea1_grupo/: se omiten los casos de la Tarea 1.')
        return filas
    esperadas = {1: 2500.0, 2: None, 3: None}
    for caso, f in esperadas.items():
        fs, datos = wavfile.read(TAREA1 / f'pruebas/caso_{caso}/03_entrada_contaminada.wav')
        assert fs == FS
        filas.append(resumen(f'Tarea 1 caso {caso}', procesar_senal(datos / 32767), f))
    return filas


def pruebas_sinteticas(rng):
    """Situaciones que el profesor puede pedir en P1-P6."""
    t = np.arange(6 * FS) / FS
    util = tres_tonos(t)
    ruido = 0.02 * rng.standard_normal(len(t))
    filas, extras = [], {}

    # P1/P6: tono fuera de bin (2513.7 Hz no es múltiplo de 15.625 Hz).
    x = util + ruido + 0.2 * np.sin(2 * np.pi * 2513.7 * t)
    filas.append(resumen('Tono 2513.7 Hz fuera de bin', procesar_senal(x), 2513.7))

    # 60 Hz y armónicos: deben agruparse como familia armónica.
    red = sum(a * np.sin(2 * np.pi * 60 * h * t) for h, a in ((1, 0.2), (3, 0.08), (5, 0.05)))
    res = procesar_senal(util + ruido + red)
    filas.append(resumen('Red 60 Hz + 3.er y 5.o armónicos', res, 60.0, tambien=(180.0, 300.0)))
    # Bloques donde 180 y 300 Hz aparecen ambos marcados como armónicos de 60 Hz.
    marcados = sum(1 for r in res
                   if sum(1 for s in r if s['flags'] & FLAG_ARMONICO) == 2)
    extras['armonicos_180_300_marcados_pct'] = round(100 * marcados / len(res), 2)

    # P4: ruido impulsivo. Un impulso es plano en frecuencia: no debe validarse nada.
    clic = np.zeros(len(t))
    clic[rng.choice(len(t), 30, replace=False)] = rng.choice([-0.8, 0.8], 30)
    filas.append(resumen('Ruido impulsivo (30 clics)', procesar_senal(util + ruido + clic), None))

    # Bandas de ruido: una angosta (100 Hz, ~6 bins) se reporta como interferente
    # alrededor de su centro; una ancha (600 Hz) no forma picos y se deja al nodo 3.
    for lo, hi in ((2950, 3050), (2700, 3300)):
        sos = butter(4, [lo, hi], btype='bandpass', fs=FS, output='sos')
        banda = sosfilt(sos, rng.standard_normal(len(t)))
        banda *= 0.15 / np.std(banda)
        res = procesar_senal(util + ruido + banda)
        filas.append(resumen(f'Ruido de banda {lo}-{hi} Hz', res, (lo + hi) / 2,
                             tol=(hi - lo) / 2 + FS / N_BLOQUE))

    # Ráfaga corta (64 ms) vs larga (640 ms): la persistencia filtra la corta.
    for dur in (0.064, 0.64):
        rafaga = np.where((t >= 2) & (t < 2 + dur), 0.2 * np.sin(2 * np.pi * 3100 * t), 0)
        res = procesar_senal(util + ruido + rafaga)
        filas.append(resumen(f'Ráfaga 3100 Hz de {int(dur*1000)} ms', res, 3100.0,
                             t_inicio=2.0, t_fin=2.0 + dur))

    # P6: el ruido cambia de 2500 a 3300 Hz a los 3 s; se mide cuánto tarda en adaptarse.
    salto = np.where(t < 3, np.sin(2 * np.pi * 2500 * t), np.sin(2 * np.pi * 3300 * t)) * 0.2
    res = procesar_senal(util + ruido + salto)
    nueva = next(i for i, r in enumerate(res) if interferentes(r, 3300.0))
    vieja = next(i for i, r in enumerate(res) if fin_bloque_s(i) > 3 and not interferentes(r, 2500.0))
    extras['adaptacion_nueva_ms'] = round(1000 * (fin_bloque_s(nueva) - 3), 1)
    extras['liberacion_vieja_ms'] = round(1000 * (fin_bloque_s(vieja) - 3), 1)
    extras['resultados_salto'] = res
    extras['senal_salto'] = util + ruido + salto
    return filas, extras


def pruebas_emisor(rng):
    """C6: el receptor debe confirmar las frecuencias que el emisor tenía que enviar.

    El emisor simulado manda los tonos de FRECUENCIAS_EMISOR con amplitudes de la
    Tarea 1 (la de 2500 Hz como 0.2) más ruido blanco. Se varían el ruido, un tono
    faltante, un error de reloj del emisor y un tono ajeno a la tabla.
    """
    t = np.arange(6 * FS) / FS
    amplitudes = dict(zip(FRECUENCIAS_EMISOR, (0.3, 0.165, 0.09, 0.2)))

    def emisor(escala=1.0, faltante=None, extra=None, sigma=0.02):
        x = sigma * rng.standard_normal(len(t))
        for f, a in amplitudes.items():
            if f != faltante:
                x += a * np.sin(2 * np.pi * f * escala * t + rng.uniform(0, 2 * np.pi))
        if extra:
            x += 0.1 * np.sin(2 * np.pi * extra * t)
        return x

    escenarios = [
        ('Nominal, ruido sigma=0.02', emisor()),
        ('Ruido fuerte, sigma=0.15', emisor(sigma=0.15)),
        ('Falta 880 Hz', emisor(faltante=880.0)),
        ('Reloj del emisor +1 %', emisor(escala=1.01)),
        ('Reloj del emisor +3 %', emisor(escala=1.03)),
        ('Tono ajeno de 3700 Hz', emisor(extra=3700.0)),
    ]
    filas = []
    for nombre, x in escenarios:
        resultados = procesar_senal(x)[2:]  # se descartan los bloques que llenan el historial
        fila = {'escenario': nombre}
        no_esp = 0
        for r in resultados:
            reporte, extra = validar_contra_emisor(r)
            no_esp += len(extra)
            for item in reporte:
                clave = f"{int(item['f_emisor_hz'])}Hz_recibida_pct"
                fila[clave] = fila.get(clave, 0) + item['recibida']
                if item['recibida']:
                    ce = f"{int(item['f_emisor_hz'])}Hz_error_max"
                    fila[ce] = max(fila.get(ce, 0), abs(item['error_hz']))
        for f in FRECUENCIAS_EMISOR:
            clave = f'{int(f)}Hz_recibida_pct'
            fila[clave] = round(100 * fila[clave] / len(resultados), 1)
        fila = {k: (round(v, 2) if 'error' in k else v) for k, v in fila.items()}
        fila['no_esperadas_por_bloque'] = round(no_esp / len(resultados), 2)
        filas.append(fila)
    # Columnas fijas: si un tono nunca se recibe, su error queda vacío.
    columnas = ['escenario'] + [c for f in FRECUENCIAS_EMISOR
                                for c in (f'{int(f)}Hz_recibida_pct', f'{int(f)}Hz_error_max')]
    columnas.append('no_esperadas_por_bloque')
    return [{c: fila.get(c, '') for c in columnas} for fila in filas]


def senal_emisor(modo):
    """Muestras del emisor real (emisor_esp32/senales.h) como códigos del DAC de 8 bits."""
    n = 65536
    t = np.arange(n) / FS
    if modo == 1:
        x = np.sin(2 * np.pi * 1000 * t)
    elif modo == 2:
        x = np.sin(2 * np.pi * 2500 * t)
    elif modo == 3:
        x = (np.sin(2 * np.pi * 440 * t) + .55 * np.sin(2 * np.pi * 880 * t + np.radians(35))
             + .30 * np.sin(2 * np.pi * 1320 * t - np.radians(50)))
    else:
        x = np.exp(-5 * np.fmod(t, 1.0)) * (np.sin(2 * np.pi * 2 * t) > 0) * np.sin(2 * np.pi * 600 * t)
    # Rampa sin^2 de 320 muestras en ambos extremos, igual que el firmware.
    borde = np.minimum(np.arange(n), n - 1 - np.arange(n))
    rampa = np.where(borde < 320, np.sin(np.pi * .5 * borde / 319) ** 2, 1.0)
    x = x * rampa
    return np.clip(np.round(128 + 80 * x / np.abs(x).max()), 0, 255)


def pruebas_modos_emisor(rng):
    """C6 con el emisor real: DAC de 8 bits (GPIO25) -> cable -> ADC de 12 bits del ESP32-S3.

    El ruido del ADC se modela como gaussiano de 3 LSB (supuesto, no medido) y,
    como caso pesimista, de 30 LSB.
    """
    filas = []
    for modo, tabla in TABLAS_POR_MODO.items():
        voltios = senal_emisor(modo) / 255 * 3.3          # salida aproximada del DAC
        for ruido_lsb in (3, 30):
            adc = np.round(voltios / 3.1 * 4095 + ruido_lsb * rng.standard_normal(len(voltios)))
            x = np.clip(adc, 0, 4095) / 4095
            res = procesar_senal(x)[3:]   # los primeros bloques llenan el historial
            recibidas = {f: 0 for f in tabla}
            error = {f: 0.0 for f in tabla}
            no_esperadas = 0
            for r in res:
                reporte, extra = validar_contra_emisor(r, tabla)
                no_esperadas += len(extra)
                for item in reporte:
                    if item['recibida']:
                        recibidas[item['f_emisor_hz']] += 1
                        error[item['f_emisor_hz']] = max(error[item['f_emisor_hz']],
                                                         abs(item['error_hz']))
            filas.append({
                'modo': modo, 'tabla_Hz': ' / '.join(f'{f:g}' for f in tabla),
                'ruido_adc_lsb': ruido_lsb,
                'recibida_pct': ' / '.join(f'{100 * recibidas[f] / len(res):.1f}' for f in tabla),
                'error_max_Hz': ' / '.join(f'{error[f]:.2f}' for f in tabla),
                'no_esperadas_por_bloque': round(no_esperadas / len(res), 2),
            })
    return filas


def curva_deteccion(rng, repeticiones=300):
    """Probabilidad de detección contra relación tono/ruido (TNR) en ruido blanco.

    TNR = (A^2/2)/sigma^2 en toda la banda. Ganancia de la FFT con Hann:
    SNR_bin = TNR + 10 log10(N/3) = TNR + 25.3 dB, así que el umbral de 20 dB
    corresponde a TNR ~ -5.3 dB (más hasta 1.4 dB de pérdida por desfase de bin).
    """
    filas = []
    n_largo = N_BLOQUE + 5 * SALTO  # 6 bloques: persistencia evaluada en el último
    for tnr in np.arange(-14, 3, 1.0):
        inst = val = 0
        for _ in range(repeticiones):
            f = rng.uniform(1500, 6500)
            a = np.sqrt(2 * 10 ** (tnr / 10))  # sigma = 1
            tt = np.arange(n_largo) / FS
            x = a * np.sin(2 * np.pi * f * tt + rng.uniform(0, 2 * np.pi)) + rng.standard_normal(n_largo)
            ok, _ = candidatos_bloque(x[:N_BLOQUE])
            inst += any(abs(c['f_hz'] - f) < 2 * FS / N_BLOQUE for c in ok)
            rec = Receptor(banda_util=(0, 0))
            for r in procesar_senal(x, rec)[-1:]:
                val += bool(interferentes(r, f))
        filas.append({'TNR_dB': tnr, 'SNR_bin_teorico_dB': round(tnr + 10 * np.log10(N_BLOQUE / 3), 2),
                      'Pd_instantanea_pct': round(100 * inst / repeticiones, 1),
                      'Pd_validada_pct': round(100 * val / repeticiones, 1)})
    # Falsas alarmas: solo ruido blanco, muchos bloques.
    bloques = 3000
    x = rng.standard_normal(N_BLOQUE + (bloques - 1) * SALTO)
    rec = Receptor(banda_util=(0, 0))
    inst = sum(len(candidatos_bloque(x[i * SALTO:i * SALTO + N_BLOQUE])[0]) for i in range(bloques))
    val = sum(len(r) for r in procesar_senal(x, rec))
    fa = {'bloques_solo_ruido': bloques, 'falsas_instantaneas': inst, 'falsas_validadas': val}
    return filas, fa


def error_interpolacion():
    """Error de frecuencia con y sin interpolación al mover un tono dentro de un bin."""
    filas = []
    k0 = 160  # 2500 Hz
    for d in np.linspace(0, 1, 21):
        f = (k0 + d) * FS / N_BLOQUE
        x = 0.5 * np.sin(2 * np.pi * f * np.arange(N_BLOQUE) / FS + 0.3)
        p = espectro_potencia(x)
        k = int(np.argmax(p))
        dd, _ = interpolar_pico(10 * np.log10(p), k)
        filas.append({'desfase_bins': round(d, 3), 'f_Hz': round(f, 4),
                      'error_sin_interp_Hz': round(abs(k * FS / N_BLOQUE - f), 4),
                      'error_con_interp_Hz': round(abs((k + dd) * FS / N_BLOQUE - f), 4)})
    return filas


def guardar_csv(ruta, filas):
    with ruta.open('w', newline='', encoding='utf-8') as archivo:
        w = csv.DictWriter(archivo, fieldnames=list(filas[0]))
        w.writeheader()
        w.writerows(filas)


def figuras(curva, interp, extras):
    plt.rcParams.update({'font.size': 10})
    # Pd vs TNR
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    tnr = [c['TNR_dB'] for c in curva]
    ax.plot(tnr, [c['Pd_instantanea_pct'] for c in curva], 'o-', label='Un bloque (C1–C3)')
    ax.plot(tnr, [c['Pd_validada_pct'] for c in curva], 's-', label='Validada (3 de 4 bloques)')
    umbral = UMBRAL_DB - 10 * np.log10(N_BLOQUE / 3)
    ax.axvline(umbral, color='gray', ls='--', lw=1)
    ax.text(umbral + 0.2, 8, f'umbral teórico\n{umbral:.1f} dB', fontsize=8, color='gray')
    ax.set_xlabel('Relación tono/ruido de banda completa, TNR (dB)')
    ax.set_ylabel('Probabilidad de detección (%)')
    ax.set_title('Detección de un tono en ruido blanco (N = 1024, Hann)')
    ax.grid(alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIGURAS / 'receptor_pd_vs_tnr.png', dpi=200)
    plt.close(fig)

    # Error de interpolación
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    d = [r['desfase_bins'] for r in interp]
    ax.plot(d, [r['error_sin_interp_Hz'] for r in interp], 'o-', label='Bin máximo')
    ax.plot(d, [r['error_con_interp_Hz'] for r in interp], 's-', label='Interpolación parabólica (dB)')
    ax.set_xlabel('Posición del tono dentro del bin (fracción de Δf = 15.625 Hz)')
    ax.set_ylabel('|error| (Hz)')
    ax.set_title('Precisión de la frecuencia estimada')
    ax.grid(alpha=0.4)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIGURAS / 'receptor_error_frecuencia.png', dpi=200)
    plt.close(fig)

    # Espectrograma con detecciones del salto 2500 -> 3300 Hz
    x, res = extras['senal_salto'], extras['resultados_salto']
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    ax.specgram(x, NFFT=N_BLOQUE, Fs=FS, noverlap=N_BLOQUE - SALTO, cmap='Greys', vmin=-90)
    for i, r in enumerate(res):
        tc = (i * SALTO + N_BLOQUE) / FS  # el resultado existe al terminar el bloque
        for s in r:
            color = 'tab:red' if s['flags'] & FLAG_INTERFERENTE else 'tab:blue'
            ax.plot(tc, s['f_hz'], '.', color=color, ms=3)
    ax.plot([], [], '.', color='tab:red', label='validada: interferente')
    ax.plot([], [], '.', color='tab:blue', label='validada: banda útil')
    ax.set_ylim(0, 4000)
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Frecuencia (Hz)')
    ax.set_title('Interferencia que cambia de 2500 a 3300 Hz en t = 3 s')
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(FIGURAS / 'receptor_salto_frecuencia.png', dpi=200)
    plt.close(fig)


def main():
    FIGURAS.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(SEMILLA)
    filas = pruebas_casos()
    sint, extras = pruebas_sinteticas(rng)
    filas += sint
    curva, fa = curva_deteccion(rng)
    interp = error_interpolacion()
    emisor = pruebas_emisor(rng)
    guardar_csv(SALIDA / 'receptor_emisor.csv', emisor)
    modos = pruebas_modos_emisor(rng)
    guardar_csv(SALIDA / 'receptor_modos_emisor.csv', modos)
    guardar_csv(SALIDA / 'receptor_pruebas.csv', filas)
    guardar_csv(SALIDA / 'receptor_pd_vs_tnr.csv', curva)
    guardar_csv(SALIDA / 'receptor_error_frecuencia.csv', interp)
    # Solo los valores escalares van al CSV; la señal y los resultados son para la figura.
    otros = {k: v for k, v in extras.items() if not isinstance(v, (list, np.ndarray))}
    guardar_csv(SALIDA / 'receptor_resumen.csv', [{**otros, **fa}])
    figuras(curva, interp, extras)
    for f in filas + emisor + modos:
        print(f)
    print(otros, fa)


if __name__ == '__main__':
    main()