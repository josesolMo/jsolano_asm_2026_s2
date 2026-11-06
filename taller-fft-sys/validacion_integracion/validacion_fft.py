"""Verificacion independiente de FFT e integracion del taller CE1110.

Ejecutar desde la raiz del repositorio o desde cualquier directorio:
    python taller-fft-sys/validacion_integracion/validacion_fft.py
No modifica los modulos implementados por otros integrantes.
"""
from __future__ import annotations

import csv
from datetime import date
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")  # Graficas y PDF sin requerir ventana interactiva.
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from scipy.signal import find_peaks, welch

DIR_MODULO = Path(__file__).resolve().parent
DIR_TALLER = DIR_MODULO.parent
DIR_SISTEMA = DIR_TALLER / "full_system"
DIR_RESULTADOS = DIR_MODULO / "resultados"
sys.path.insert(0, str(DIR_SISTEMA))

from sistema_ruido.config import FS, N, L, BIN_HZ  # noqa: E402
from sistema_ruido.scenarios import escenario  # noqa: E402
from sistema_ruido.signals import fade, generar_senal  # noqa: E402
from sistema_ruido.pipeline import correr_prueba  # noqa: E402

METODOS = ("FIR", "IIR", "ESPECTRAL")
TOLERANCIA_HZ = BIN_HZ


def detectar_picos_fft(x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Detecta picos sin usar la lista de frecuencias esperadas.

    Se toma un bloque de N muestras, se elimina DC y se aplica Hann; se
    conservan los picos que superan el 18% de la magnitud dominante.
    """
    x = np.asarray(x, dtype=float)
    if x.shape != (N,):
        raise ValueError(f"Se requieren exactamente {N} muestras; recibidas {x.size}")
    ventana = np.hanning(N)
    magnitud = np.abs(np.fft.rfft((x - x.mean()) * ventana))
    frecuencias = np.fft.rfftfreq(N, d=1 / FS)
    indices, _ = find_peaks(magnitud, distance=2)
    if indices.size and magnitud.max() > 0:
        indices = indices[magnitud[indices] >= 0.18 * magnitud.max()]
    indices = indices[np.argsort(magnitud[indices])[::-1]]
    return frecuencias, magnitud, indices


def validar_componentes(x: np.ndarray, esperadas: list[float], nombre: str,
                        inicio: int = 0, tolerancia_hz: float = TOLERANCIA_HZ) -> list[dict]:
    """Compara picos detectados con el plan de frecuencias, con margen de 1 bin.

    Las frecuencias esperadas se usan solo despues de detectar los picos; no se
    busca el maximo dentro de una ventana centrada en la respuesta esperada.
    """
    x = np.asarray(x, dtype=float)
    if inicio < 0 or inicio + N > len(x):
        raise ValueError("El tramo seleccionado cae fuera de la senal")
    freqs, _, indices = detectar_picos_fft(x[inicio:inicio + N])
    candidatos = freqs[indices]
    filas = []
    for f0 in esperadas:
        if len(candidatos):
            f1 = float(candidatos[np.argmin(np.abs(candidatos - f0))])
            err = abs(float(f0) - f1)
        else:
            f1, err = float("nan"), float("inf")
        filas.append(dict(modo=nombre, frecuencia_esperada_hz=float(f0),
                          frecuencia_detectada_hz=f1, error_hz=err,
                          tolerancia_hz=float(tolerancia_hz),
                          cumple=bool(err <= tolerancia_hz)))
    return filas


def validar_modos() -> list[dict]:
    """Valida las 5 modalidades nominales del emisor sobre senales sinteticas.

    Se usa el mismo generador de referencia que el proyecto, no una captura
    electrica del ESP32. El modo melodia se inspecciona por nota.
    """
    t = np.arange(L) / FS
    casos = [
        ("1 - seno 1000", dict(tipo="multitono", f=[1000], a=[1], fase=[0]), [1000], L // 2),
        ("2 - seno 2500", dict(tipo="multitono", f=[2500], a=[1], fase=[0]), [2500], L // 2),
        ("3 - multitono", escenario("P2")["senal"], [440, 880, 1320], L // 2),
        ("4 - transitoria", escenario("P4")["senal"], [600], int(.08 * FS)),
    ]
    filas = []
    for nombre, parametros, esperadas, inicio in casos:
        x = fade(generar_senal(t=t, **parametros))
        filas.extend(validar_componentes(x, esperadas, nombre, inicio))
    notas = escenario("P3")["senal"]["notas"]
    x = fade(generar_senal(t=t, **escenario("P3")["senal"]))
    longitud_nota = L // len(notas)
    for i, nota in enumerate(notas):
        filas.extend(validar_componentes(x, [nota], f"5 - melodia nota {i + 1}",
                                         i * longitud_nota + int(.12 * FS)))
    return filas


def integrar_escenarios() -> tuple[list[dict], dict]:
    """Ejecuta el pipeline REAL del repositorio para P1, P2 y P5.

    P5 se registra para ilustrar el solapamiento intencional: una FFT no puede
    separar dos componentes que ocupan exactamente la misma frecuencia.
    """
    metricas = []
    resultados = {}
    for pid in ("P1", "P2", "P5"):
        res = correr_prueba(escenario(pid))
        resultados[pid] = res
        for metodo in METODOS:
            m = res["metricas"][metodo]
            fila = dict(prueba=pid, metodo=metodo, snr_entrada_db=m["snr_in"],
                        snr_salida_db=m["snr_out"], mejora_snr_db=m["dsnr"],
                        atenuacion_interferencia_db=m["atenuacion"],
                        energia_util_pct=m["e_util"])
            valores = [fila[k] for k in list(fila)[2:]]
            if not np.all(np.isfinite(valores)):
                raise RuntimeError(f"Metricas no finitas en {pid} / {metodo}")
            metricas.append(fila)
    return metricas, resultados


def escribir_csv(ruta: Path, filas: list[dict]) -> None:
    if not filas:
        raise ValueError("No hay filas para exportar")
    with ruta.open("w", newline="", encoding="utf-8-sig") as archivo:
        escritor = csv.DictWriter(archivo, fieldnames=list(filas[0]))
        escritor.writeheader()
        escritor.writerows(filas)


def generar_figuras(resultados: dict) -> dict[str, Path]:
    rutas = {}
    for pid, res in resultados.items():
        fig, ax = plt.subplots(figsize=(10, 5.2))
        senales = [("Referencia limpia", res["limpia"]),
                   ("ADC simulado", res["adc"])]
        senales += [(m, res["salidas"][m]) for m in METODOS]
        for nombre, datos in senales:
            f, p = welch(datos, fs=FS, nperseg=2048)
            ax.plot(f, 10 * np.log10(p + 1e-16), label=nombre, lw=1.2)
        ax.axvspan(*res["esc"]["banda_util"], color="green", alpha=.09,
                   label="Banda util")
        ax.axvspan(*res["esc"]["rango_int"], color="red", alpha=.06,
                   label="Banda interferente")
        ax.set(title=f"CE1110 - Integracion {pid}: {res['esc']['nombre']}",
               xlabel="Frecuencia (Hz)", ylabel="Densidad espectral (dB/Hz)",
               xlim=(0, FS / 2))
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, ncol=2)
        fig.tight_layout()
        ruta = DIR_RESULTADOS / f"espectro_{pid}.png"
        fig.savefig(ruta, dpi=150)
        plt.close(fig)
        rutas[pid] = ruta
    return rutas


def construir_resumen(filas_fft: list[dict], metricas: list[dict]) -> list[str]:
    n_ok = sum(f["cumple"] for f in filas_fft)
    lineas = [
        "# Validacion FFT e integracion - CE1110",
        "",
        "**Alcance:** comprobacion digital del prototipo Python y del pipeline existente.",
        f"**Configuracion:** fs = {FS} Hz; N = {N}; resolucion = {BIN_HZ:.3f} Hz/bin; "
        f"tolerancia = +/- {TOLERANCIA_HZ:.3f} Hz.",
        f"**Resultado espectral:** {n_ok}/{len(filas_fft)} frecuencias dentro de tolerancia.",
        "",
        "## Criterios de validacion",
        "",
        "- Se seleccionan 1024 muestras, se elimina la componente continua y se aplica ventana Hann.",
        "- Se detectan picos locales del espectro real con magnitud >=18 % del pico dominante.",
        "- Una frecuencia cumple si uno de los picos detectados difiere, como maximo, un bin FFT.",
        "- Se ejecuta el pipeline original del equipo para P1, P2 y P5 sin modificarlo.",
        "- Las metricas reportadas son los valores calculados por el codigo del proyecto.",
        "",
        "## Deteccion de componentes",
        "",
        "| Modo | Esperada (Hz) | Detectada (Hz) | Error (Hz) | Cumple |",
        "|---|---:|---:|---:|:---:|",
    ]
    for f in filas_fft:
        med = f"{f['frecuencia_detectada_hz']:.3f}" if np.isfinite(f["frecuencia_detectada_hz"]) else "sin pico"
        lineas.append(f"| {f['modo']} | {f['frecuencia_esperada_hz']:.3f} | {med} | "
                      f"{f['error_hz']:.3f} | {'Si' if f['cumple'] else 'No'} |")
    lineas += ["", "## Metricas de integracion", "",
               "| Caso | Filtro | SNR entrada (dB) | SNR salida (dB) | Mejora (dB) | Atenuacion (dB) |",
               "|---|---|---:|---:|---:|---:|"]
    for m in metricas:
        lineas.append(f"| {m['prueba']} | {m['metodo']} | {m['snr_entrada_db']:.2f} | "
                      f"{m['snr_salida_db']:.2f} | {m['mejora_snr_db']:.2f} | "
                      f"{m['atenuacion_interferencia_db']:.2f} |")
    lineas += [
        "", "## Discusion", "",
        "P1 evalua la separacion entre el tono util de 1000 Hz y un interferente de 2500 Hz.",
        "P2 examina tres tonos utiles (440, 880 y 1320 Hz) ante ruido de alta frecuencia.",
        "P5 incluye interferencia dentro de la propia banda util; su eliminacion puede "
        "degradar la componente de 1320 Hz. No se exige mejora positiva en todos los filtros.",
        "", "## Limitaciones y coherencia con el hardware", "",
        "- Todas estas evidencias son simuladas; no certifican captura real por ADC ni salida analogica.",
        "- El emisor ESP32 del equipo utiliza DAC de 8 bits (GPIO25), mientras que el prototipo "
        "de alto nivel simula cuantizacion DAC/ADC de 12 bits: falta una prueba cruzada "
        "de resolucion, ganancia y posible distorsion analogica.",
        "- fs = 16000 Hz es nominal: se requiere osciloscopio o adquisicion externa "
        "para medir la tasa de muestreo efectiva de la placa.",
        "- Los archivos CSV y PNG son parte de la evidencia reproducible.",
        "", "## Referencias internas", "",
        "- `taller-fft-sys/full_system/sistema_ruido/{config,signals,scenarios,pipeline,metrics}.py`.",
        "- `taller-fft-sys/emisor_esp32/{senales.h,emisor_esp32.ino}`.",
        "- Taller de sesion asincronica, semana 12, CE1110 (21-30 octubre 2026).", "",
    ]
    return lineas


def generar_pdf(filas_fft: list[dict], metricas: list[dict], rutas_figuras: dict[str, Path]) -> Path:
    """Crea un ANEXO PDF autonomo para insertar en la documentacion grupal."""
    ruta = DIR_RESULTADOS / "anexo_validacion_fft.pdf"
    with PdfPages(ruta, metadata={"Title": "CE1110 - Anexo de validacion FFT", "Author": "Integrante 4"}) as pdf:
        fig = plt.figure(figsize=(8.27, 11.69))
        fig.text(.07, .95, "CE1110 | Analisis de Senales Mixtas", fontsize=17, weight="bold")
        fig.text(.07, .917, "Taller semana 12 - Anexo de validacion FFT", fontsize=13)
        fig.text(.07, .883, f"Fecha de entrega: 30/10/2026   |   fs = {FS} Hz   |   N = {N}", fontsize=10)
        descripcion = [
            "Objetivo: contrastar frecuencias del emisor con picos FFT",
            "y obtener evidencias del procesamiento FIR, IIR y espectral.",
            "", "Metodo:",
            "  1. Generar referencias con los modulos del equipo.",
            "  2. Seleccionar tramos de 1024 muestras (64 ms).",
            "  3. Eliminar DC, aplicar Hann y calcular rFFT.",
            "  4. Detectar picos antes de consultar el plan esperado.",
            f"  5. Aceptar errores de frecuencia <= {TOLERANCIA_HZ:.3f} Hz (1 bin).",
            "  6. Correr pipeline existente para P1, P2 y P5.",
            "", "Resultados de frecuencias (prototipo digital):",
        ]
        y = .83
        for linea in descripcion:
            fig.text(.08, y, linea, fontsize=10, family="monospace")
            y -= .028
        fig.text(.08, y - .008, "Modo                      Esperado   Detectado   Error   OK", fontsize=9, family="monospace", weight="bold")
        y -= .037
        for f in filas_fft:
            linea = (f"{f['modo'][:24]:24s} {f['frecuencia_esperada_hz']:8.2f} "
                     f"{f['frecuencia_detectada_hz']:10.2f} {f['error_hz']:7.2f} "
                     f"{'SI' if f['cumple'] else 'NO'}")
            fig.text(.08, y, linea, fontsize=8.2, family="monospace")
            y -= .026
        total = sum(f["cumple"] for f in filas_fft)
        fig.text(.08, y - .025, f"Componentes dentro de tolerancia: {total}/{len(filas_fft)}", weight="bold", fontsize=11)
        fig.text(.08, .11, "Nota: pruebas de simulacion; no hay mediciones fisicas del DAC ni del ADC.", fontsize=9)
        fig.text(.08, .08, "DAC real: 8 bits; modelo ADC/DAC del prototipo: 12 bits (limitacion).", fontsize=9)
        pdf.savefig(fig)
        plt.close(fig)

        fig = plt.figure(figsize=(8.27, 11.69))
        fig.text(.07, .94, "Metricas del pipeline original", weight="bold", fontsize=17)
        fig.text(.07, .905, "Casos P1, P2 y P5: comparacion de FIR, IIR y espectral", fontsize=10)
        y = .86
        fig.text(.07, y, "Prueba   Filtro      SNRin   SNRout  Mejora   Atenuacion", fontsize=9, weight="bold", family="monospace")
        y -= .029
        for m in metricas:
            linea = (f"{m['prueba']:6s}   {m['metodo']:10s}  {m['snr_entrada_db']:6.2f}  "
                     f"{m['snr_salida_db']:6.2f}  {m['mejora_snr_db']:6.2f}  "
                     f"{m['atenuacion_interferencia_db']:8.2f}")
            fig.text(.07, y, linea, fontsize=9, family="monospace")
            y -= .03
        notas = [
            "Discusion y alcance:",
            "P1: seno util 1000 Hz e interferente tonal 2500 Hz.",
            "P2: tonos 440 / 880 / 1320 Hz y ruido de alta frecuencia.",
            "P5: solapamiento intencional entre tono util y ruido.",
            "Es esperable que algunos filtros degraden componentes utiles.",
            "", "Advertencias:",
            "- No se garantiza mejora de SNR en cada caso.",
            "- Esta prueba usa el pipeline simulado, no una captura de ADC.",
            "- La tasa de DAC del ESP32 se declara, pero no se mide aqui.",
            "- Discrepancia de resolucion: emisor fisico 8 bits vs modelo 12 bits.",
        ]
        y -= .06
        for linea in notas:
            fig.text(.08, y, linea, fontsize=10)
            y -= .036
        pdf.savefig(fig)
        plt.close(fig)

        for pid in rutas_figuras:
            res = plt.imread(rutas_figuras[pid])
            fig = plt.figure(figsize=(11.69, 8.27))
            fig.text(.055, .94, f"Evidencia espectral - {pid}", fontsize=16, weight="bold")
            ax = fig.add_axes((.05, .1, .9, .8))
            ax.imshow(res)
            ax.axis("off")
            pdf.savefig(fig)
            plt.close(fig)
    return ruta


def main() -> None:
    DIR_RESULTADOS.mkdir(exist_ok=True, parents=True)
    print(f"Sistema: fs={FS} Hz, N={N}, bin={BIN_HZ:.3f} Hz")
    filas_fft = validar_modos()
    escribir_csv(DIR_RESULTADOS / "frecuencias_fft.csv", filas_fft)
    for f in filas_fft:
        estado = "OK" if f["cumple"] else "ERROR"
        print(f"{estado:5s} {f['modo']:24s} esperado={f['frecuencia_esperada_hz']:8.2f} Hz "
              f"detectado={f['frecuencia_detectada_hz']:8.2f} Hz")
    print("\nEjecutando la integracion con el pipeline original...")
    metricas, resultados = integrar_escenarios()
    escribir_csv(DIR_RESULTADOS / "metricas_integracion.csv", metricas)
    for m in metricas:
        print(f"{m['prueba']} {m['metodo']:10s}: SNRin={m['snr_entrada_db']:7.2f} dB, "
              f"SNRout={m['snr_salida_db']:7.2f} dB, mejora={m['mejora_snr_db']:7.2f} dB")
    figuras = generar_figuras(resultados)
    (DIR_RESULTADOS / "reporte_validacion.md").write_text(
        "\n".join(construir_resumen(filas_fft, metricas)), encoding="utf-8")
    pdf = generar_pdf(filas_fft, metricas, figuras)
    print(f"\nReporte, CSV y graficas en: {DIR_RESULTADOS}")
    print(f"Anexo PDF: {pdf}")
    if not all(f["cumple"] for f in filas_fft):
        raise SystemExit("Fallo al menos una frecuencia: revisar frecuencias_fft.csv")
    print("VALIDACION DIGITAL COMPLETADA (no equivale a validacion fisica)")


if __name__ == "__main__":
    main()
