#!/usr/bin/env python3
"""Interfaz local minima: lee el monitor serial del Nodo 3 y muestra las lineas CSV "N3,..." en una tabla.

Uso: python nodo3_esp32/tools/monitor_nodo3.py /dev/ttyUSB0 [--plot]
En la consola de la placa (115200 baud) envie "stream on". Los demas comandos se escriben aqui tambien:
cada linea que teclee se reenvia a la placa (p. ej. "set alfa 4", "scen P2", "metrics").
"""
import sys, threading, time
import serial

COLS = ("N3,ms,bloques,perdidos,crc,desb_cola,vacios_dac,saturadas,us_prom,us_max,aten_db,util_pct,snr_est_in,snr_est_out,"
        "anillo,mse,snr_in,snr_out,dsnr,e_util,aten_fir,util_fir,snr_est_out_fir,mse_fir,dsnr_fir,escucha").split(",")[1:]
ESCUCHA = {0: "filter (espectral, Nodo 3)", 1: "bypass", 2: "raw (sin filtrar)", 3: "fir (temporal, Nodo 2)"}


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    ser = serial.Serial(sys.argv[1], 115200, timeout=0.2)
    plot = "--plot" in sys.argv
    hist = {k: [] for k in COLS}
    threading.Thread(target=lambda: [ser.write((l + "\n").encode()) for l in sys.stdin], daemon=True).start()
    if plot:
        import matplotlib.pyplot as plt
        plt.ion(); fig, ax = plt.subplots(2, 1, sharex=True)
    ult = 0
    while True:
        l = ser.readline().decode(errors="replace").strip()
        if not l:
            continue
        if not l.startswith("N3,"):
            print(l); continue
        v = l.split(",")[1:]
        if len(v) != len(COLS):
            continue
        for k, x in zip(COLS, v):
            hist[k].append(float(x))
        print(f"[{ESCUCHA.get(int(float(v[-1])), '?')}] " + "  ".join(f"{k}={x}" for k, x in zip(COLS, v) if k in
              ("bloques", "perdidos", "crc", "us_prom", "aten_db", "aten_fir", "util_pct", "util_fir",
               "snr_est_out", "snr_est_out_fir", "dsnr", "dsnr_fir")))
        if plot and time.time() - ult > 1:
            ult = time.time(); ax[0].cla(); ax[1].cla()
            fig.suptitle(f"Parlante (GPIO25): {ESCUCHA.get(int(hist['escucha'][-1]), '?')}  |  GPIO26: FIR (Nodo 2)")
            ax[0].plot(hist["us_prom"], label="us/bloque prom"); ax[0].plot(hist["us_max"], label="max"); ax[0].axhline(64000, ls="--", c="r"); ax[0].legend()
            ax[1].plot(hist["aten_db"], label="atenuacion rango, espectral [dB]")
            ax[1].plot(hist["aten_fir"], label="atenuacion rango, FIR [dB]")
            ax[1].plot(hist["snr_est_out"], "--", label="SNR est. salida, espectral")
            ax[1].plot(hist["snr_est_out_fir"], "--", label="SNR est. salida, FIR"); ax[1].legend(fontsize=8)
            plt.pause(0.01)


if __name__ == "__main__":
    main()
