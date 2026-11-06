"""Punto de entrada y coordinacion de las pruebas, metricas y salidas."""
from __future__ import annotations

import csv
import numpy as np
from .config import (DIR_SALIDA, FS, N, BIN_HZ, ADC_BITS, DAC_BITS, N_BLOQUES)
from .scenarios import escenario, separacion_hz, validar_frecuencias, chequeo_aliasing
from .pipeline import correr_prueba
from .verification import verificar_bloques, resumen_filtros
from .link import probar_enlace, disenar_enlace
from .metrics import costo_estimado
from .plots import fig_mapa_frecuencias, fig_filtros, fig_espectros

REPETICIONES = 3
METODOS = ("FIR", "IIR", "ESPECTRAL")
CAMPOS = ("mse", "snr_in", "snr_out", "dsnr", "e_util", "atenuacion")


def main():
    print("=" * 78)
    print(f"fs = {FS} Hz | N = {N} (potencia de 2) | bloque = {1e3 * N / FS:.0f} ms | "
          f"resolucion = {BIN_HZ:.3f} Hz | ADC/DAC = {ADC_BITS}/{DAC_BITS} bits")
    print("=" * 78)

    # a) Aliasing y enlace
    al = chequeo_aliasing()
    print(f"\nAliasing: Nyquist = {al['nyquist']:.0f} Hz; la mayor frecuencia util ({al['f_max_util']:.0f} Hz) "
          f"queda muy por debajo.\n  Una componente en {al['f_pliega']:.0f} Hz se pliega sobre {al['f_max_util']:.0f} Hz y "
          f"habria que atenuarla ~{al['atenuacion_requerida_db']:.0f} dB; un RC de 1er orden (fc = fs/4) da solo "
          f"{al['rc_1er_orden_db']:.0f} dB.\n  -> La fuente (emisor DAC/PWM con su filtro de reconstruccion) debe estar "
          f"limitada en banda; un tono de 9 kHz se veria en {al['ejemplo_9k_se_pliega_en']:.0f} Hz.")
    en = disenar_enlace()
    print(f"\nEnlace (UART 8N1, bloques int16): trama = {en['bytes_trama']} B (overhead {en['overhead_pct']:.2f} %), "
          f"flujo = {en['bits_s'] / 1e3:.0f} kbit/s -> baud minimo {en['baud_min'] / 1e3:.0f} k "
          f"-> usar {en['baud']} baud (utilizacion {100 * en['utilizacion']:.0f} %, "
          f"trama en {en['ms_por_trama']:.1f} ms de {en['ms_bloque']:.0f} ms).")

    # b) Plan de frecuencias
    print("\nPlan de frecuencias por prueba")
    print(f"  {'id':3s} {'banda util (Hz)':>17s} {'interferencia (Hz)':>20s} {'separacion':>11s}  validacion")
    for pid in ("P1", "P2", "P3", "P4", "P5"):
        e = escenario(pid)
        sep = separacion_hz(e["banda_util"], e["rango_int"])
        av = validar_frecuencias(e)
        estado = "OK" + (" (solape intencional)" if e["solape"] else "") if not av else "; ".join(av)
        print(f"  {pid:3s} {str(tuple(int(v) for v in e['banda_util'])):>17s} "
              f"{str(tuple(int(v) for v in e['rango_int'])):>20s} {sep:8.0f} Hz  {estado}")

    # c) Pruebas P1-P5 (3 repeticiones) y tres sorteos independientes de P6
    grupos = [(pid, [correr_prueba(escenario(pid, r)) for r in range(REPETICIONES)])
              for pid in ("P1", "P2", "P3", "P4", "P5")]
    grupos += [(f"P6.{r + 1}", [correr_prueba(escenario("P6", r))]) for r in range(3)]
    resultados, tabla = {pid: reps[0] for pid, reps in grupos}, []
    verificar_bloques(resultados["P1"])
    for pid, reps in grupos:
        e0 = reps[0]["esc"]
        fil = resumen_filtros(reps[0])
        print(f"\n{pid}: {e0['nombre']}  (SNR objetivo {e0['snr_db']:.1f} dB, banda util {e0['banda_util']})")
        print(f"   FIR orden {fil['fir']['orden']} | IIR eliptico orden {fil['iir']['orden']} "
              f"(radio polo max {fil['iir']['radio_polo_max']:.4f})")
        print(f"   {'metodo':10s}" + "".join(f"{c:>17s}" for c in CAMPOS))
        for m in METODOS:
            fila, celdas = {"prueba": pid, "metodo": m, "repeticiones": len(reps)}, []
            for c in CAMPOS:
                v = np.array([r["metricas"][m][c] for r in reps])
                esc_ = 1e3 if c == "mse" else 1.0
                sd = v.std(ddof=1) if len(v) > 1 else 0.0
                fila[c + "_media"], fila[c + "_std"] = float(v.mean()), float(sd)
                celdas.append(f"{v.mean() * esc_:8.2f}+-{sd * esc_:5.2f}" if len(v) > 1 else f"{v.mean() * esc_:8.2f}")
            tabla.append(fila)
            print(f"   {m:10s}" + "".join(f"{c:>17s}" for c in celdas))
    print("   (MSE en unidades de 1e-3 de la escala completa; SNR y atenuacion en dB; "
          "media+-desv. sobre 3 repeticiones; P6.x = un sorteo)")

    with open(DIR_SALIDA / "metricas.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(tabla[0]))
        w.writeheader()
        w.writerows(tabla)

    # d) Filtros de la prueba P5, protocolo, costo
    fil = resumen_filtros(resultados["P5"])
    print("\nFiltros candidatos (P5: paso hasta 1420 Hz, rechazo desde 2120 Hz)")
    print(f"  FIR  orden {fil['fir']['orden']}, retardo {fil['fir']['retardo_muestras']} muestras, rizado "
          f"{fil['fir']['rizado_db']:.3f} dB, rechazo {fil['fir']['aten_db']:.1f} dB "
          f"({fil['fir']['aten_q15_db']:.1f} dB con coeficientes Q15, |b|max = {fil['fir']['max_coef']:.3f})")
    print(f"  IIR  orden {fil['iir']['orden']} ({fil['iir']['biquads']} biquads, la ultima de 1er orden si el orden es impar), rizado {fil['iir']['rizado_db']:.3f} dB, "
          f"rechazo {fil['iir']['aten_db']:.1f} dB, radio de polo max {fil['iir']['radio_polo_max']:.5f} "
          f"(float32: {fil['iir']['radio_polo_max_float32']:.5f}) -> estable; |coef|max = {fil['iir']['max_coef']:.2f} "
          f"(no cabe en Q15: usar float32 o Q2.13)")

    rng = np.random.default_rng(99)
    bloques = np.round(resultados["P5"]["adc"].reshape(N_BLOQUES, N) * 32767).astype(np.int16)
    enlace = probar_enlace(bloques, ber=1e-5, p_perdida=0.05, rng=rng)
    print(f"\nPrueba del enlace (BER 1e-5, 5 % de tramas perdidas): {enlace['enviadas']} enviadas, "
          f"{enlace['perdidas_canal']} perdidas en el canal, {enlace['recibidas_ok']} validas, "
          f"{enlace['descartadas_por_error']} descartadas por CRC/cabecera, "
          f"{enlace['aceptadas_con_error']} aceptadas con datos erroneos.\n"
          f"  El receptor detecta {len(enlace['seq_faltantes'])} bloques faltantes por salto de secuencia: "
          f"{enlace['seq_faltantes']}")

    print("\nCosto estimado por bloque (aprox., float32)")
    print(f"  {'nodo':34s}{'Mflop/s':>10s}{'memoria (B)':>13s}{'latencia (ms)':>15s}")
    for f in costo_estimado(resultados["P5"]["b"], resultados["P5"]["sos"]):
        print(f"  {f['nodo']:34s}{f['mflops']:10.2f}{f['mem_B']:13.0f}{f['lat_ms']:15.1f}")

    # e) Figuras
    fig_mapa_frecuencias(DIR_SALIDA / "01_mapa_frecuencias.png")
    fig_filtros(resultados["P5"], DIR_SALIDA / "02_filtros_P5.png")
    fig_espectros(resultados, DIR_SALIDA / "03_espectros_por_prueba.png")
    print(f"\nFiguras y metricas.csv guardados en {DIR_SALIDA.resolve()}")
