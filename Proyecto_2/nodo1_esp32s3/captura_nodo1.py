"""PC del paso 1: recibe los bloques del nodo 1 y los pasa por el receptor de Python.

Uso (desde taller-fft-sys/, con la placa conectada):
    python nodo1_esp32s3/captura_nodo1.py --puerto COM5 --segundos 5 --modo 3
Analizar una captura guardada, sin placa:
    python nodo1_esp32s3/captura_nodo1.py --archivo nodo1_esp32s3/capturas/captura.bin --modo 3

--modo es el modo del emisor (1 = 1000 Hz, 2 = 2500 Hz, 3 = 440/880/1320 Hz,
4 = transitoria de 600 Hz). Requiere pyserial para leer la placa: pip install pyserial
"""
import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.io import wavfile

CARPETA = Path(__file__).resolve().parent
TALLER = CARPETA.parent
# config.py de full_system crea "taller-fft-sys/resultados_sistema" relativo a la
# carpeta actual, asi que link.py se importa parado en la raiz del repositorio.
_actual = Path.cwd()
os.chdir(TALLER.parent)
sys.path[:0] = [str(TALLER / "full_system"), str(TALLER)]
from sistema_ruido.link import desempaquetar_flujo  # noqa: E402
os.chdir(_actual)
from receptor_fft.src.receptor_fft import (  # noqa: E402
    FS, N_BLOQUE, TABLAS_POR_MODO, procesar_senal, validar_contra_emisor)

BLOQUES_HISTORIAL = 3   # los primeros resultados del receptor llenan su historial (C4)


def leer_puerto(puerto, segundos, baud):
    """Lee bytes crudos del puerto durante 'segundos' y pide el estado al final."""
    import serial  # se importa aqui para poder analizar archivos sin pyserial

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = puerto, baud, 0.1
    # Bajar DTR/RTS antes de abrir evita reiniciar la placa en muchos adaptadores.
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(1.0)            # si la placa se reinicio igual, se le da tiempo
    ser.reset_input_buffer()   # se descarta lo que llego mientras arrancaba
    datos = bytearray()
    fin = time.time() + segundos
    while time.time() < fin:
        datos += ser.read(8192)
    ser.write(b"e")            # el firmware responde con una linea "#estado ..."
    time.sleep(0.3)
    datos += ser.read(8192)
    ser.close()
    return bytes(datos)


def reconstruir(flujo):
    """Valida tramas (SYNC, longitud, CRC) y las ordena por secuencia."""
    tramas = desempaquetar_flujo(flujo)
    if not tramas:
        raise SystemExit("No llego ninguna trama valida. Revise puerto, placa y cable.")
    secuencias = sorted(tramas)
    # La secuencia es de 16 bits; en capturas cortas no da la vuelta.
    faltantes = sorted(set(range(secuencias[0], secuencias[-1] + 1)) - set(secuencias))
    codigos = np.concatenate([tramas[s] for s in secuencias]).astype(np.int32)
    estado = [l for l in flujo.decode("latin-1").splitlines() if l.startswith("#estado")]
    return codigos, len(secuencias), faltantes, estado


def informe_senal(codigos):
    """Revisa nivel de DC, excursion y recortes del ADC de 12 bits."""
    vmax = 3.1  # rango aproximado con atenuacion de 12 dB
    print(f"Muestras: {len(codigos)} ({len(codigos) / FS:.2f} s a {FS} Hz)")
    print(f"Codigos: min {codigos.min()}, media {codigos.mean():.0f}, max {codigos.max()} "
          f"(~{codigos.min() / 4095 * vmax:.2f} V a {codigos.max() / 4095 * vmax:.2f} V)")
    recortes = int(np.sum((codigos <= 0) | (codigos >= 4095)))
    if recortes:
        print(f"ATENCION: {recortes} muestras en el limite del ADC: la senal se recorta.")
    if not 1200 < codigos.mean() < 2900:
        print("ATENCION: la media no esta cerca del centro; revise el divisor o la tierra comun.")


def informe_receptor(x, modo):
    """Corre el receptor sobre la captura y la compara con la tabla del modo."""
    tabla = TABLAS_POR_MODO[modo]
    resultados = procesar_senal(x)[BLOQUES_HISTORIAL:]
    if not resultados:
        raise SystemExit("La captura es muy corta para el receptor (minimo ~0.2 s).")
    recibidas = {f: [] for f in tabla}
    no_esperadas = []
    for r in resultados:
        reporte, extra = validar_contra_emisor(r, tabla)
        no_esperadas += [round(e['f_hz'], 1) for e in extra]
        for item in reporte:
            if item['recibida']:
                recibidas[item['f_emisor_hz']].append(item['error_hz'])
    print(f"\nReceptor, modo {modo} (tabla {', '.join(f'{f:g}' for f in tabla)} Hz), "
          f"{len(resultados)} bloques:")
    for f, errores in recibidas.items():
        if errores:
            print(f"  {f:7.1f} Hz: recibida en {100 * len(errores) / len(resultados):5.1f} % "
                  f"de los bloques, error medio {np.mean(errores):+.2f} Hz")
        else:
            print(f"  {f:7.1f} Hz: AUSENTE")
    if no_esperadas:
        valores, cuentas = np.unique(no_esperadas, return_counts=True)
        principales = sorted(zip(cuentas, valores), reverse=True)[:5]
        print("  No esperadas (Hz: bloques): "
              + ", ".join(f"{v:g}: {c}" for c, v in principales))
    else:
        print("  Ninguna frecuencia no esperada.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    fuente = p.add_mutually_exclusive_group(required=True)
    fuente.add_argument("--puerto", help="puerto serial de la placa (COM5, /dev/ttyACM0...)")
    fuente.add_argument("--archivo", help="captura .bin guardada antes")
    p.add_argument("--segundos", type=float, default=5.0)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--modo", type=int, choices=sorted(TABLAS_POR_MODO), default=3)
    args = p.parse_args()

    salida = CARPETA / "capturas"
    salida.mkdir(exist_ok=True)
    if args.puerto:
        flujo = leer_puerto(args.puerto, args.segundos, args.baud)
        marca = time.strftime("%Y%m%d_%H%M%S")
        (salida / f"captura_{marca}.bin").write_bytes(flujo)
    else:
        flujo = Path(args.archivo).read_bytes()
        marca = Path(args.archivo).stem

    codigos, n_tramas, faltantes, estado = reconstruir(flujo)
    print(f"Tramas validas: {n_tramas} de {N_BLOQUE} muestras; faltantes por secuencia: "
          f"{len(faltantes)}{' ' + str(faltantes[:10]) if faltantes else ''}")
    for linea in estado:
        print(linea)
    informe_senal(codigos)

    # El receptor quita el DC por bloque; basta con llevar los codigos a [0, 1].
    x = codigos / 4095.0
    np.save(salida / f"captura_{marca}.npy", codigos)
    wavfile.write(salida / f"captura_{marca}.wav", FS,
                  np.int16(np.clip((x - x.mean()) * 2, -1, 1) * 32767))
    informe_receptor(x, args.modo)
    print(f"\nGuardado en {salida}")


if __name__ == "__main__":
    main()
