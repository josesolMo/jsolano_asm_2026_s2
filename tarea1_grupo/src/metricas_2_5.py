"""CE1110 — Tarea 1, sección 2.5: métricas y figuras reproducibles.

Antes de integrar: py tarea1_grupo/src/metricas_2_5.py --temporal D:/ASM/asm_temporal/tarea1_grupo --espectral D:/ASM/asm_espectral/tarea1_grupo --benchmark
Después de integrar: py tarea1_grupo/src/metricas_2_5.py --base tarea1_grupo --benchmark
Solo figuras: py tarea1_grupo/src/metricas_2_5.py --solo-figuras
No cambia GitHub ni sobrescribe resultados/metricas.csv.
"""

import argparse
import csv
import hashlib
import importlib.util
import math
import statistics
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile

FS = 16000
TONOS = (440, 880, 1320)
N_FIR = 1024
N_FFT = 1024
HOP_FFT = N_FFT // 2
RETARDO_FIR = 20
RECORTE = RETARDO_FIR


def leer_wav(ruta):
    if not ruta.is_file():
        raise FileNotFoundError(f"No se encontró: {ruta}")
    fs, x = wavfile.read(ruta)
    if fs != FS or x.ndim != 1 or x.dtype != np.int16:
        raise ValueError(f"Formato WAV inesperado en {ruta}: {fs} Hz, {x.shape}, {x.dtype}")
    return x.astype(np.float64) / 32767.0


def verificar_mismas_entradas(t, e):
    for c in range(1, 4):
        for nombre in ("01_referencia_limpia.wav", "03_entrada_contaminada.wav"):
            a = t / "pruebas" / f"caso_{c}" / nombre
            b = e / "pruebas" / f"caso_{c}" / nombre
            sha_a = hashlib.sha256(a.read_bytes()).hexdigest()
            sha_b = hashlib.sha256(b.read_bytes()).hexdigest()
            if sha_a != sha_b:
                raise ValueError(f"Las ramas no comparten el mismo archivo {nombre} del caso {c}")


def potencia(x):
    return float(np.mean(np.square(x)))


def mse_snr(ref, x):
    error = potencia(x - ref)
    return error, float(10 * math.log10(potencia(ref) / error)) if error else math.inf


def proyectar_tonos(x, instante_inicial):
    """Amplitudes a frecuencias exactas, compensando ciclos no enteros en el intervalo."""
    tiempos = (np.arange(len(x)) + instante_inicial) / FS
    cols = []
    for f in TONOS:
        w = 2 * np.pi * f * tiempos
        cols.extend((np.sin(w), np.cos(w)))
    M = np.column_stack(cols)
    parametros = np.linalg.lstsq(M, x, rcond=None)[0]
    return np.hypot(parametros[0::2], parametros[1::2])


def amplitud_tono(x, f, instante_inicial):
    tiempos = (np.arange(len(x)) + instante_inicial) / FS
    w = 2 * np.pi * f * tiempos
    M = np.column_stack([np.sin(w), np.cos(w)])
    par = np.linalg.lstsq(M, x, rcond=None)[0]
    return float(np.hypot(*par))


def reduccion_error_banda(ref, contaminada, salida, f1, f2):
    """Atenuación de potencia del ERROR residual en una banda, no de la salida bruta."""
    ventana = np.hanning(len(ref))
    freqs = np.fft.rfftfreq(len(ref), 1 / FS)
    mascara = (freqs >= f1) & (freqs <= f2)
    pe = float(np.sum(np.abs(np.fft.rfft((contaminada - ref) * ventana))[mascara] ** 2))
    ps = float(np.sum(np.abs(np.fft.rfft((salida - ref) * ventana))[mascara] ** 2))
    return 10 * math.log10(pe / ps) if ps > 0 else math.inf


def cargar_modulo(nombre, ruta):
    spec = importlib.util.spec_from_file_location(nombre, ruta)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"No se puede cargar {ruta}")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def tiempo_en_pc(t, e, entrada):
    """Mediana de 3 ejecuciones, normalizada por bloques/hops procesados.

    El procesamiento espectral se mide por archivo completo. Su implementación
    actual no es aún un motor streaming para ESP32; esta cifra NO predice la
    latencia ni el costo real por hop en el microcontrolador.
    """
    temporal = cargar_modulo("fir_25_local", t / "src" / "filtros_temporales.py")
    espectral = cargar_modulo("fft_25_local", e / "src" / "filtrado_espectral.py")
    b = temporal.disenar_filtro()

    def ejecutar_fir():
        estado = None
        for ini in range(0, len(entrada), N_FIR):
            _, estado = temporal.filtrar_manual(entrada[ini:ini + N_FIR], b, estado)

    def ejecutar_fft():
        espectral.reducir_ruido(entrada)

    def mediana(func):
        func()  # calentamiento
        corridas = []
        for _ in range(3):
            inicio = time.perf_counter()
            func()
            corridas.append(time.perf_counter() - inicio)
        return statistics.median(corridas)

    bloques_fir = math.ceil(len(entrada) / N_FIR)
    longitud_padding = len(entrada) + 2 * HOP_FFT
    longitud_padding += (-(longitud_padding - N_FFT)) % HOP_FFT
    bloques_fft = 1 + (longitud_padding - N_FFT) // HOP_FFT
    return {"Temporal": 1000 * mediana(ejecutar_fir) / bloques_fir,
            "Espectral": 1000 * mediana(ejecutar_fft) / bloques_fft}


def crear_figuras(filas, destino):
    """Tres figuras propias de la sección 2.5, basadas en el mismo CSV."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Falta matplotlib: py -m pip install matplotlib") from exc

    destino.mkdir(parents=True, exist_ok=True)
    por_caso = {(int(f["caso"]), f["metodo"]): f for f in filas}
    casos = (1, 2, 3)
    metodos = ("Temporal", "Espectral")
    etiquetas = ("Caso 1", "Caso 2", "Caso 3")
    x = np.arange(3)
    ancho = 0.36

    # 1. Ganancia de SNR respecto a la entrada contaminada.
    fig, ax = plt.subplots(figsize=(8.2, 4.5))
    for i, metodo in enumerate(metodos):
        valores = [float(por_caso[c, metodo]["delta_SNR_dB"]) for c in casos]
        ax.bar(x + (i - .5) * ancho, valores, width=ancho, label=metodo)
    ax.set_xticks(x, etiquetas)
    ax.set_ylabel("Mejora de SNR (dB)")
    ax.set_title("Sección 2.5: mejora de SNR por caso")
    ax.legend()
    ax.grid(axis="y", alpha=.25)
    fig.tight_layout()
    fig.savefig(destino / "comparativa_snr_2_5.png", dpi=170)
    plt.close(fig)

    # 2. Conservación de los tres tonos útiles: caso 3 (ruido superpuesto).
    fig, ax = plt.subplots(figsize=(8.2, 4.5))
    x_t = np.arange(3)
    nombres = ("440 Hz", "880 Hz", "1320 Hz")
    cols = ("energia_440_Hz_pct", "energia_880_Hz_pct", "energia_1320_Hz_pct")
    for i, metodo in enumerate(("Entrada",) + metodos):
        vals = [float(por_caso[3, metodo][col]) for col in cols]
        ax.bar(x_t + (i - 1) * .26, vals, width=.26, label=metodo)
    ax.axhline(100, color="gray", linestyle="--", linewidth=1, label="Referencia 100 %")
    ax.set_xticks(x_t, nombres)
    ax.set_ylabel("Energía estimada relativa (%)")
    ax.set_title("Caso 3: energía de tonos útiles conservada")
    ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(.5, -.12), ncol=4)
    ax.grid(axis="y", alpha=.25)
    fig.tight_layout()
    fig.savefig(destino / "energia_tonos_caso3_2_5.png", dpi=170, bbox_inches="tight")
    plt.close(fig)

    # 3. Coste por bloque en Python/PC, si el CSV contiene benchmarking.
    disponibles = all(por_caso[c, metodo].get("tiempo_host_ms_bloque", "")
                      for c in casos for metodo in metodos)
    if disponibles:
        fig, ax = plt.subplots(figsize=(8.2, 4.5))
        for metodo in metodos:
            tiempos = [float(por_caso[c, metodo]["tiempo_host_ms_bloque"]) for c in casos]
            ax.plot(x, tiempos, marker="o", linewidth=2, label=metodo)
        ax.set_xticks(x, etiquetas)
        ax.set_ylabel("Tiempo en Python/PC por bloque (ms)")
        ax.set_title("Sección 2.5: tiempo de ejecución en computadora")
        ax.legend()
        ax.grid(alpha=.25)
        fig.tight_layout()
        fig.savefig(destino / "tiempos_pc_2_5.png", dpi=170)
        plt.close(fig)
    else:
        print("Figura de tiempos omitida: falta ejecutar con --benchmark.")


def main():
    ap = argparse.ArgumentParser(description="Métricas y figuras comparables de la sección 2.5")
    ap.add_argument("--base", type=Path, help="Carpeta tarea1_grupo ya integrada: contiene ambos filtros")
    ap.add_argument("--temporal", type=Path, help="Carpeta tarea1_grupo de la rama temporal")
    ap.add_argument("--espectral", type=Path, help="Carpeta tarea1_grupo de la rama espectral")
    ap.add_argument("--out", type=Path, help="CSV de salida (por defecto resultados/metricas_2_5.csv)")
    ap.add_argument("--figuras", type=Path, help="Directorio donde escribir tres PNG")
    ap.add_argument("--solo-figuras", action="store_true", help="Generar gráficos desde el CSV existente")
    ap.add_argument("--sin-figuras", action="store_true", help="No crear gráficos")
    ap.add_argument("--benchmark", action="store_true", help="Medir tiempos orientativos en Python/PC")
    a = ap.parse_args()

    base = (a.base or Path(__file__).resolve().parents[1]).resolve()
    out = a.out or base / "resultados" / "metricas_2_5.csv"
    figuras = a.figuras or base / "resultados" / "figuras"
    if a.solo_figuras:
        with out.open(encoding="utf-8-sig", newline="") as archivo:
            crear_figuras(list(csv.DictReader(archivo)), figuras)
        print(f"Figuras guardadas en {figuras.resolve()}")
        return

    if bool(a.temporal) != bool(a.espectral):
        ap.error("Indica ambas rutas --temporal y --espectral, o únicamente --base")
    t = (a.temporal or base).resolve()
    e = (a.espectral or base).resolve()
    verificar_mismas_entradas(t, e)
    registros = []
    for caso in (1, 2, 3):
        prueba = t / "pruebas" / f"caso_{caso}"
        limpia = leer_wav(prueba / "01_referencia_limpia.wav")
        sucia = leer_wav(prueba / "03_entrada_contaminada.wav")
        fir = leer_wav(t / "resultados" / "filtrado_temporal" / f"caso_{caso}" / "04_filtrada_temporal.wav")
        fft = leer_wav(e / "resultados" / "filtrado_espectral" / f"caso_{caso}" / "04_filtrada_espectral.wav")
        if not (len(limpia) == len(sucia) == len(fir) == len(fft) == 96000):
            raise ValueError(f"Longitud inesperada: caso {caso} debe contener 96000 muestras")
        ref = limpia[RECORTE:-RECORTE]
        x = sucia[RECORTE:-RECORTE]
        senales = {"Entrada": x,
                   "Temporal": fir[2 * RETARDO_FIR:],
                   "Espectral": fft[RECORTE:-RECORTE]}
        if not all(len(v) == len(ref) for v in senales.values()):
            raise ValueError(f"Error de alineación de señales, caso {caso}")

        amp_ref = proyectar_tonos(ref, RECORTE)
        _, snr_base = mse_snr(ref, x)
        tiempos = tiempo_en_pc(t, e, sucia) if a.benchmark else {}
        for modo, y in senales.items():
            mse, snr = mse_snr(ref, y)
            amp = proyectar_tonos(y, RECORTE)
            conservada = 100 * amp ** 2 / amp_ref ** 2
            amp_2500_entrada = amplitud_tono(x, 2500, RECORTE)
            amp_2500_salida = amplitud_tono(y, 2500, RECORTE)
            atenuacion_2500 = (20 * math.log10(amp_2500_entrada / amp_2500_salida)
                              if amp_2500_salida > 0 else math.inf) if caso == 1 else ""
            banda = {1: (2400, 2600), 2: (2000, 8000), 3: (1000, 2200)}[caso]
            atenuacion_banda = reduccion_error_banda(ref, x, y, *banda)
            r = {
                "caso": caso, "metodo": modo, "fs_Hz": FS,
                "muestras_comparadas": len(ref), "MSE": mse,
                "SNR_dB": snr, "delta_SNR_dB": snr - snr_base,
                "energia_util_tonos_pct": float(100 * np.dot(amp, amp) / np.dot(amp_ref, amp_ref)),
                "energia_440_Hz_pct": float(conservada[0]),
                "energia_880_Hz_pct": float(conservada[1]),
                "energia_1320_Hz_pct": float(conservada[2]),
                "atenuacion_tono_2500_dB": atenuacion_2500,
                "atenuacion_error_banda_dB": atenuacion_banda,
                "banda_error_Hz": f"{banda[0]}-{banda[1]}",
                "tiempo_host_ms_bloque": tiempos.get(modo, ""),
                "tipo_tiempo": "Python PC (indicativo)" if modo in tiempos else "no medido",
                "formato_comparacion": "PCM16 WAV, alineacion FIR 20 muestras",
            }
            registros.append(r)
            print(f"Caso {caso} | {modo:9} | MSE={mse:.7g} | SNR={snr:8.3f} dB | ΔSNR={snr-snr_base:+8.3f} dB | tono 1320={conservada[2]:.2f}%")

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as archivo:
        writer = csv.DictWriter(archivo, fieldnames=registros[0].keys())
        writer.writeheader()
        writer.writerows(registros)
    if not a.sin_figuras:
        crear_figuras(registros, figuras)
        print(f"Figuras guardadas en {figuras.resolve()}")
    print(f"CSV generado: {out.resolve()}")
    if a.benchmark:
        print("NOTA: los tiempos medidos son de Python/PC, no del ESP32-S3.")


if __name__ == "__main__":
    main()
