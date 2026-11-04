#!/usr/bin/env python3
"""Emula al Nodo 1 para probar el Nodo 3 sin ese hardware.

Genera el escenario Pn del proyecto (misma senal que full_system), lo cuantiza como el ADC y lo
envia en tramas PCM (link.empaquetar) a ritmo real, mas las tramas de parametros (receptor_fft)
y, opcionalmente, la extension PISO. Con --ref-port envia la senal limpia (referencia) por otro
puerto, con la misma numeracion, para las metricas verdaderas.

Ejemplos (desde taller-fft-sys/, con numpy, scipy y pyserial):
  python nodo3_esp32/tools/simular_nodo1.py --port /dev/ttyUSB1 --escenario P1
  python nodo3_esp32/tools/simular_nodo1.py --port /dev/ttyUSB1 --ref-port /dev/ttyUSB2 --escenario P5 --piso
  python nodo3_esp32/tools/simular_nodo1.py --out flujo.bin --escenario P2      # sin puerto: vuelca el flujo
Conexion: adaptador USB-UART 3.3 V: TX -> GPIO16 (RX2) de la ESP32 del Nodo 3, GND comun.
"""
import argparse, os, struct, sys, time
from pathlib import Path
import numpy as np

RAIZ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ)); sys.path.insert(0, str(RAIZ / "full_system"))
os.chdir(RAIZ.parent)
from sistema_ruido import link, scenarios, signals                         # noqa: E402
from sistema_ruido.config import FS, L, N, N_BLOQUES, PICO, ADC_BITS         # noqa: E402
from sistema_ruido.acquisition import NodoAdquisicion                        # noqa: E402
from receptor_fft.src import receptor_fft as rf                              # noqa: E402


def trama_piso(sec, piso, decim=8):
    cdb = np.round(100 * 10 * np.log10(np.maximum(piso[::decim], 1e-30))).astype("<i2")
    cuerpo = bytes([0xA5, 0x5A, 0x03, sec & 0xFF, decim, len(cdb)]) + cdb.tobytes()
    return cuerpo + struct.pack("<H", link.crc16(cuerpo))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--escenario", default="P1", choices=["P1", "P2", "P3", "P4", "P5", "P6"])
    ap.add_argument("--port"); ap.add_argument("--ref-port"); ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--out", help="escribe el flujo PCM+parametros en un archivo en vez de un puerto")
    ap.add_argument("--piso", action="store_true", help="envia tambien la trama PISO (extension propuesta)")
    ap.add_argument("--sin-params", action="store_true")
    ap.add_argument("--repeticiones", type=int, default=1)
    a = ap.parse_args()
    if not a.port and not a.out:
        ap.error("indique --port o --out")

    esc = scenarios.escenario(a.escenario)
    rng = np.random.default_rng(esc["semilla"])
    t = np.arange(L) / FS
    limpia = signals.fade(signals.generar_senal(t=t, **esc["senal"]))
    ruido = signals.escalar_para_snr(limpia, signals.generar_ruido(n=L, rng=rng, **esc["ruido"]), esc["snr_db"])
    g = PICO / max(np.abs(limpia).max(), np.abs(limpia + ruido).max())
    limpia, ruidosa = g * limpia, g * (limpia + ruido)
    adc = np.round(signals.cuantizar(ruidosa, ADC_BITS) * 32768).astype("<i2")      # Q15
    ref = np.round(limpia * 32768).astype("<i2")
    print(f"{a.escenario}: {esc['nombre']} | SNR de entrada {esc['snr_db']} dB | banda util {esc['banda_util']} Hz | "
          f"rango de interferencia {esc['rango_int']} Hz", file=sys.stderr)

    nodo1, receptor = NodoAdquisicion(esc["banda_util"]), rf.Receptor(banda_util=esc["banda_util"])
    ser = ser_ref = None
    if a.port:
        import serial
        ser = serial.Serial(a.port, a.baud)
        if a.ref_port:
            ser_ref = serial.Serial(a.ref_port, a.baud)
    salida = open(a.out, "wb") if a.out else None
    seq = 0
    t0 = time.perf_counter()
    for rep in range(a.repeticiones):
        for i in range(N_BLOQUES):
            bloque = adc[i * N:(i + 1) * N]
            x = bloque.astype(np.float64) / 32768.0
            diag = nodo1.procesar(x)
            datos = link.empaquetar(seq, bloque)
            if not a.sin_params:
                frec = []
                for h in range(2):                       # receptor_fft analiza ventanas de N con avance N/2
                    ini = i * N + (h - 1) * (N // 2)
                    if ini >= 0:
                        frec = receptor.procesar(adc[ini:ini + N].astype(np.float64) / 32768.0)
                datos += rf.empaquetar(seq, frec)
            if a.piso:
                datos += trama_piso(seq, diag.piso)
            if salida:
                salida.write(datos)
            else:
                # ritmo real: un bloque cada 64 ms
                espera = t0 + (rep * N_BLOQUES + i) * N / FS - time.perf_counter()
                if espera > 0:
                    time.sleep(espera)
                ser.write(datos)
                if ser_ref:
                    ser_ref.write(link.empaquetar(seq, ref[i * N:(i + 1) * N]))
            seq = (seq + 1) & 0xFFFF
    print(f"enviados {seq} bloques ({seq * N / FS:.2f} s de senal)", file=sys.stderr)


if __name__ == "__main__":
    main()
