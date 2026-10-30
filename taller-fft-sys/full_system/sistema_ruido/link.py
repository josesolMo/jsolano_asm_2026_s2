"""Tramas del enlace cableado, CRC-16 y simulacion de transmision."""
from __future__ import annotations

import struct
import numpy as np
from .config import N, FS


SYNC = b"\xA5\x5A"


ENCABEZADO = 6          # SYNC(2) + secuencia(2) + muestras(2)


CRC_BYTES = 2


def _tabla_crc16() -> list[int]:
    tabla = []
    for b in range(256):
        c = b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        tabla.append(c)
    return tabla


_TABLA = _tabla_crc16()


def crc16(datos: bytes, c: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poli 0x1021, init 0xFFFF). Detecta toda rafaga de errores <= 16 bits."""
    for byte in datos:
        c = ((c << 8) & 0xFFFF) ^ _TABLA[(c >> 8) ^ byte]
    return c


def empaquetar(seq: int, muestras: np.ndarray) -> bytes:
    """[SYNC][seq u16][n u16][n x int16][CRC16 de seq+n+datos], little-endian."""
    cuerpo = struct.pack("<HH", seq & 0xFFFF, len(muestras)) + muestras.astype("<i2").tobytes()
    return SYNC + cuerpo + struct.pack("<H", crc16(cuerpo))


def desempaquetar_flujo(flujo: bytes) -> dict[int, np.ndarray]:
    """Recorre un flujo de bytes, se resincroniza buscando SYNC y devuelve {seq: muestras} validas."""
    validas, i = {}, 0
    while True:
        i = flujo.find(SYNC, i)
        if i < 0 or i + ENCABEZADO > len(flujo):
            return validas
        seq, n = struct.unpack_from("<HH", flujo, i + 2)
        fin = i + ENCABEZADO + 2 * n + CRC_BYTES
        if n == N and fin <= len(flujo):
            (crc_rx,) = struct.unpack_from("<H", flujo, fin - CRC_BYTES)
            if crc16(flujo[i + 2: fin - CRC_BYTES]) == crc_rx:
                validas[seq] = np.frombuffer(flujo[i + ENCABEZADO: fin - CRC_BYTES], dtype="<i2")
                i = fin
                continue
        i += 1                                   # falso sync o trama danada: seguir buscando


def disenar_enlace(utilizacion_max: float = 0.8) -> dict:
    """Tasa de comunicacion minima para un enlace UART 8N1 que transporta bloques de N muestras int16."""
    bytes_trama = 2 * N + ENCABEZADO + CRC_BYTES
    bits_s = bytes_trama * 10 / (N / FS)         # 10 bits por byte en 8N1
    baud_min = bits_s / utilizacion_max
    baud = next(b for b in (115_200, 230_400, 460_800, 921_600, 1_000_000, 2_000_000) if b >= baud_min)
    return dict(bytes_trama=bytes_trama, overhead_pct=100 * (ENCABEZADO + CRC_BYTES) / (2 * N),
                bits_s=bits_s, baud_min=baud_min, baud=baud, utilizacion=bits_s / baud,
                ms_por_trama=bytes_trama * 10 / baud * 1e3, ms_bloque=1e3 * N / FS)


def probar_enlace(bloques_int16: np.ndarray, ber: float, p_perdida: float, rng) -> dict:
    """Simula errores de bit y perdida de tramas; verifica que ninguna trama danada se acepte."""
    n = len(bloques_int16)
    tramas = [empaquetar(i, b) for i, b in enumerate(bloques_int16)]
    perdidas = {i for i in range(n) if rng.random() < p_perdida}
    flujo = np.frombuffer(b"".join(t for i, t in enumerate(tramas) if i not in perdidas), dtype=np.uint8)
    bits = np.unpackbits(flujo)
    bits ^= (rng.random(bits.size) < ber).astype(np.uint8)
    rx = desempaquetar_flujo(np.packbits(bits).tobytes())
    erroneas = sum(1 for s, m in rx.items() if not np.array_equal(m, bloques_int16[s]))
    return dict(enviadas=n, perdidas_canal=len(perdidas), recibidas_ok=len(rx),
                descartadas_por_error=n - len(perdidas) - len(rx),
                seq_faltantes=sorted(set(range(n)) - set(rx)), aceptadas_con_error=erroneas)
