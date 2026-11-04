"""PC de los pasos 2 y 3: valida la FFT y el receptor que corren en el ESP32-S3.

La placa envia tres tipos de trama (ver trama.h):
  A5 5A  muestras crudas (1024 por trama)
  A5 5B  diagnostico: frecuencias validadas por el receptor en C++ (cada 32 ms)
  A5 5C  espectro de un bloque, a pedido con el comando 'f'

Este script:
  1. compara el espectro calculado en la placa con numpy (valida la FFT);
  2. corre el receptor de Python sobre las mismas muestras y compara, bloque a
     bloque, sus frecuencias con las de la placa (valida el receptor en C++);
  3. resume lo que detecto la placa y, con --modo, lo compara con el emisor.

Uso (desde la carpeta que contiene nodo1_esp32s3/, con la placa conectada):
    python nodo1_esp32s3/validar_nodo1.py --puerto COM6 --segundos 4 --modo 3
Sin placa, con una captura guardada:
    python nodo1_esp32s3/validar_nodo1.py --archivo nodo1_esp32s3/capturas/validacion_<fecha>.bin
"""
import argparse
import struct
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np

CARPETA = Path(__file__).resolve().parent
sys.path.insert(0, str(CARPETA.parent))
from receptor_fft.src.receptor_fft import (  # noqa: E402
    FLAG_ARMONICO, FLAG_BANDA_UTIL, FLAG_INTERFERENTE, N_BLOQUE, SALTO, TABLAS_POR_MODO,
    Receptor, espectro_potencia, validar_contra_emisor)

CALENTAMIENTO = 4    # analisis que tarda el historial de Python en igualar al de la placa (C4)


def crc16(datos, crc=0xFFFF):
    """CRC-16/CCITT-FALSE, igual que trama.h y link.py."""
    for byte in datos:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def leer_tramas(flujo):
    """Recorre el flujo, se resincroniza con A5 y valida el CRC de cada trama."""
    muestras, diagnosticos, espectros = {}, {}, {}
    i = 0
    while True:
        i = flujo.find(b"\xA5", i)
        if i < 0 or i + 2 > len(flujo):
            break
        tipo = flujo[i + 1]
        try:
            if tipo == 0x5A:
                seq, n = struct.unpack_from("<HH", flujo, i + 2)
                largo = 6 + 2 * n + 2
                if n == N_BLOQUE and i + largo <= len(flujo) and \
                        crc16(flujo[i + 2:i + largo - 2]) == struct.unpack_from("<H", flujo, i + largo - 2)[0]:
                    muestras[seq] = np.frombuffer(flujo[i + 6:i + largo - 2], dtype="<i2").astype(int)
                    i += largo
                    continue
            elif tipo == 0x5B:
                analisis, t_us, n = struct.unpack_from("<IHB", flujo, i + 2)
                largo = 2 + 8 + 7 * n + 2
                if n <= 8 and i + largo <= len(flujo) and \
                        crc16(flujo[i + 2:i + largo - 2]) == struct.unpack_from("<H", flujo, i + largo - 2)[0]:
                    frecs = []
                    for k in range(n):
                        f4, amp, snr, flags = struct.unpack_from("<HHhB", flujo, i + 10 + 7 * k)
                        frecs.append((f4 / 4, amp / 10000, snr / 10, flags))
                    diagnosticos[analisis] = (t_us, frecs)
                    i += largo
                    continue
            elif tipo == 0x5C:
                analisis, n = struct.unpack_from("<IH", flujo, i + 2)
                largo = 2 + 6 + 4 * n + 4 * (n // 2 + 1) + 2
                if n == N_BLOQUE and i + largo <= len(flujo) and \
                        crc16(flujo[i + 2:i + largo - 2]) == struct.unpack_from("<H", flujo, i + largo - 2)[0]:
                    x = np.frombuffer(flujo[i + 8:i + 8 + 4 * n], dtype="<f4").astype(float)
                    p = np.frombuffer(flujo[i + 8 + 4 * n:i + largo - 2], dtype="<f4").astype(float)
                    espectros[analisis] = (x, p)
                    i += largo
                    continue
        except struct.error:
            break   # trama cortada al final de la captura
        i += 1      # falso sync o trama danada: seguir buscando
    estado = [l for l in flujo.decode("latin-1").splitlines() if l.startswith("#estado")]
    return muestras, diagnosticos, espectros, estado


def leer_puerto(puerto, segundos, baud):
    """Lee el puerto 'segundos', pide un espectro al inicio y el estado al final."""
    import serial  # pip install pyserial

    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = puerto, baud, 0.1
    # Bajar DTR/RTS antes de abrir evita reiniciar la placa en muchos adaptadores.
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(1.0)
    ser.reset_input_buffer()
    ser.write(b"f")            # la placa envia el espectro del proximo bloque
    datos = bytearray()
    fin = time.time() + segundos
    while time.time() < fin:
        datos += ser.read(16384)
    ser.write(b"e")
    time.sleep(0.3)
    datos += ser.read(16384)
    ser.close()
    return bytes(datos)


def validar_fft(espectros):
    """Paso 2: el espectro de la placa frente a numpy con la misma entrada."""
    if not espectros:
        print("FFT: no llego ninguna trama de espectro (comando 'f').")
        return None
    peor = 0.0
    for x, p in espectros.values():
        ref = espectro_potencia(x)
        peor = max(peor, float(np.max(np.abs(p - ref)) / max(np.max(ref), 1e-30)))
    estado = "OK" if peor < 1e-4 else "REVISAR"
    print(f"FFT de la placa frente a numpy: error relativo maximo {peor:.2e} [{estado}]")
    return peor < 1e-4


def tramo_continuo(muestras):
    """El tramo mas largo de tramas de muestras consecutivas."""
    secuencias = sorted(muestras)
    mejor, actual = [], [secuencias[0]]
    for s in secuencias[1:]:
        if s == actual[-1] + 1:
            actual.append(s)
        else:
            mejor, actual = max(mejor, actual, key=len), [s]
    return max(mejor, actual, key=len)


def validar_receptor(muestras, diagnosticos):
    """Paso 3: frecuencias de la placa frente al receptor de Python, bloque a bloque."""
    tramo = tramo_continuo(muestras)
    x = np.concatenate([muestras[s] for s in tramo]) / 4095.0
    inicio = tramo[0] * N_BLOQUE           # indice global de la primera muestra
    rx = Receptor()
    iguales = comparados = 0
    for j, desde in enumerate(range(0, len(x) - N_BLOQUE + 1, SALTO)):
        py = rx.procesar(x[desde:desde + N_BLOQUE])
        analisis = (inicio + desde) // SALTO   # el analisis a cubre [512 a, 512 a + 1024)
        if j < CALENTAMIENTO or analisis not in diagnosticos:
            continue
        placa = diagnosticos[analisis][1]
        comparados += 1
        # La trama lleva la frecuencia en cuartos de Hz: tolerancia de 0.2 Hz.
        if len(py) == len(placa) and all(
                a['flags'] == b[3] and abs(a['f_hz'] - b[0]) < 0.2 for a, b in zip(py, placa)):
            iguales += 1
    if comparados == 0:
        print("Receptor: no hay bloques comparables (captura muy corta o sin diagnosticos).")
        return None
    estado = "OK" if iguales >= 0.95 * comparados else "REVISAR"
    print(f"Receptor de la placa frente a Python: {iguales}/{comparados} bloques iguales [{estado}]")
    return iguales >= 0.95 * comparados


def resumen_placa(diagnosticos, modo):
    """Lo que detecto la placa: frecuencias mas frecuentes y tiempo de analisis."""
    tiempos = [t for t, _ in diagnosticos.values()]
    print(f"\nAnalisis recibidos: {len(diagnosticos)}; tiempo por bloque en la placa: "
          f"medio {np.mean(tiempos) / 1000:.2f} ms, maximo {max(tiempos) / 1000:.2f} ms (de 32 ms)")
    conteo = Counter()
    for _, frecs in diagnosticos.values():
        for f, _, _, flags in frecs:
            tipo = ("interferente" + (", armonico" if flags & FLAG_ARMONICO else "")
                    if flags & FLAG_INTERFERENTE else "util" if flags & FLAG_BANDA_UTIL else "?")
            conteo[(round(f), tipo)] += 1
    if conteo:
        print("Frecuencias validadas por la placa (Hz, tipo: bloques):")
        for (f, tipo), c in conteo.most_common(8):
            print(f"  {f:6d} Hz, {tipo}: {c}")
    else:
        print("La placa no valido ninguna frecuencia.")
    if modo is None:
        return
    tabla = TABLAS_POR_MODO[modo]
    recibidas = Counter()
    no_esperadas = Counter()
    for _, frecs in diagnosticos.values():
        validadas = [{'f_hz': f, 'amplitud': a} for f, a, _, _ in frecs]
        reporte, extra = validar_contra_emisor(validadas, tabla)
        for item in reporte:
            recibidas[item['f_emisor_hz']] += item['recibida']
        for e in extra:
            no_esperadas[round(e['f_hz'])] += 1
    total = len(diagnosticos)
    print(f"Contra la tabla del modo {modo} del emisor:")
    for f in tabla:
        print(f"  {f:7.1f} Hz: recibida en {100 * recibidas[f] / total:5.1f} % de los bloques")
    if no_esperadas:
        print("  No esperadas (Hz: bloques): "
              + ", ".join(f"{f}: {c}" for f, c in no_esperadas.most_common(5)))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    fuente = p.add_mutually_exclusive_group(required=True)
    fuente.add_argument("--puerto", help="puerto serial de la placa (COM6, /dev/ttyUSB0...)")
    fuente.add_argument("--archivo", help="captura .bin guardada antes")
    p.add_argument("--segundos", type=float, default=4.0)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--modo", type=int, choices=sorted(TABLAS_POR_MODO),
                   help="modo del emisor, para comparar con su tabla (opcional)")
    args = p.parse_args()

    if args.puerto:
        flujo = leer_puerto(args.puerto, args.segundos, args.baud)
        carpeta = CARPETA / "capturas"
        carpeta.mkdir(exist_ok=True)
        archivo = carpeta / f"validacion_{time.strftime('%Y%m%d_%H%M%S')}.bin"
        archivo.write_bytes(flujo)
        print(f"Captura guardada en {archivo}")
    else:
        flujo = Path(args.archivo).read_bytes()

    muestras, diagnosticos, espectros, estado = leer_tramas(flujo)
    print(f"Tramas validas: {len(muestras)} de muestras, {len(diagnosticos)} de diagnostico, "
          f"{len(espectros)} de espectro")
    for linea in estado:
        print(linea)
    if not muestras or not diagnosticos:
        raise SystemExit("Faltan tramas de muestras o de diagnostico: revise firmware y puerto.")

    ok_fft = validar_fft(espectros)
    ok_rx = validar_receptor(muestras, diagnosticos)
    resumen_placa(diagnosticos, args.modo)
    if ok_fft is False or ok_rx is False:
        sys.exit(1)


if __name__ == "__main__":
    main()