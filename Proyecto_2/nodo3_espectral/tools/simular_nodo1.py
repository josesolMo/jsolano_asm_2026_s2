#!/usr/bin/env python3
"""Emula al Nodo 1 para probar el Nodo 3 sin ese hardware.

Genera el escenario Pn del proyecto (misma senal que full_system), lo cuantiza como el ADC y lo
envia en tramas PCM (link.empaquetar) a ritmo real, mas las tramas de diagnostico A5 5B (formato de nodo1_esp32s3/trama.h)
y, opcionalmente, la extension PISO (A5 5D). Con --ref-port envia la senal limpia (referencia) por otro
puerto, con la misma numeracion, para las metricas verdaderas.

Ejemplos (desde la raiz del repositorio, con numpy, scipy y pyserial):
  python Proyecto_2/nodo3_espectral/tools/simular_nodo1.py --port /dev/ttyUSB1 --escenario P1
  python Proyecto_2/nodo3_espectral/tools/simular_nodo1.py --port /dev/ttyUSB1 --ref-port /dev/ttyUSB2 --escenario P5 --piso
  python Proyecto_2/nodo3_espectral/tools/simular_nodo1.py --out flujo.bin --escenario P2      # sin puerto: vuelca el flujo
Formato por defecto: cuenta cruda del ADC (0..4095), como el firmware actual del Nodo 1.
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


def _cerrar(sync1, cuerpo):
    """Igual que trama::cerrar de nodo1_esp32s3/trama.h: SYNC + cuerpo + CRC16(cuerpo)."""
    return bytes([0xA5, sync1]) + cuerpo + struct.pack("<H", link.crc16(cuerpo))


def trama_diag(analisis, t_us, frec):
    """Diagnostico del Nodo 1 (A5 5B), mismo formato que trama::empaquetarDiagnostico."""
    sat = lambda v, lo, hi: int(round(min(max(v, lo), hi)))
    cuerpo = struct.pack("<IHBB", analisis, t_us, len(frec), 0)
    for f in frec:
        cuerpo += struct.pack("<HHhB", sat(f["f_hz"] * 4, 0, 65535), sat(f["amplitud"] * 10000, 0, 65535),
                              sat(f["snr_local_db"] * 10, -32768, 32767), f["flags"])
    return _cerrar(0x5B, cuerpo)


def trama_piso(analisis, piso, decim=8):
    """EXTENSION propuesta (A5 5D): piso de ruido en centi-dB cada 'decim' bins."""
    cdb = np.round(100 * 10 * np.log10(np.maximum(piso[::decim], 1e-30))).astype("<i2")
    return _cerrar(0x5D, struct.pack("<IBB", analisis, decim, len(cdb)) + cdb.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--escenario", default="P1", choices=["P1", "P2", "P3", "P4", "P5", "P6"])
    ap.add_argument("--port"); ap.add_argument("--ref-port"); ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", help="escribe el flujo PCM+parametros en un archivo en vez de un puerto")
    ap.add_argument("--formato", choices=["adc12", "q15"], default="adc12", help="muestras int16: cuenta cruda del ADC (como el Nodo 1 actual) o Q15 centrado")
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
    adc_f = signals.cuantizar(ruidosa, ADC_BITS)
    adc = (np.round(adc_f * 2048) + 2048).astype("<i2") if a.formato == "adc12" else np.round(adc_f * 32768).astype("<i2")
    adc_q15 = np.round(adc_f * 32768).astype("<i2")                                  # para el analisis del Nodo 1 simulado
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
            x = adc_q15[i * N:(i + 1) * N].astype(np.float64) / 32768.0
            diag = nodo1.procesar(x)
            datos = link.empaquetar(seq, bloque)
            if not a.sin_params:
                for h in range(2):                       # un analisis cada 512 muestras (32 ms), como el Nodo 1
                    ini = i * N + (h - 1) * (N // 2)
                    if ini >= 0:
                        frec = receptor.procesar(adc_q15[ini:ini + N].astype(np.float64) / 32768.0)
                        datos += trama_diag(2 * i + h, 4000, frec)
            if a.piso:
                datos += trama_piso(2 * i + 1, diag.piso)
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
