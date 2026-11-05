#!/usr/bin/env python3
"""Grafica una captura del Nodo 3: la entrada y las DOS salidas (espectral del Nodo 3 y FIR del Nodo 2)
tal como las calculo el microcontrolador, con el piso de ruido y los interferentes que estaba usando.

La PC solo dibuja y resume (visualizacion y evidencia, permitido por el enunciado); el filtrado
ocurre en la placa.

Uso (cerrar antes el monitor serie del nodo 3):
    python nodo3_esp32/tools/graficar_captura.py COM7                 # 4 bloques (256 ms)
    python nodo3_esp32/tools/graficar_captura.py COM7 --bloques 8
    python nodo3_esp32/tools/graficar_captura.py --archivo nodo3_esp32/capturas/captura_....bin

Guarda la trama (.bin) y la figura (.png) en nodo3_esp32/capturas/.
Figura: (1) formas de onda completas, (2) acercamiento de 30 ms, (3) espectros con el piso del
Nodo 1, la banda util (verde), el rango de interferencia (rojo) y los interferentes (lineas).
"""
import argparse
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

N, NB, MAX_INTERF, ENC = 1024, 513, 64, 20
ESCUCHA = {0: "filter (espectral)", 1: "bypass", 2: "raw", 3: "fir (temporal)"}
PERFIL = {0: "tono", 1: "multitono", 2: "audio", 3: "transitoria"}
CARPETA = Path(__file__).resolve().parents[1] / "capturas"


def crc16(datos: bytes) -> int:
    c = 0xFFFF
    for b in datos:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def tamano(n):
    return ENC + 2 * MAX_INTERF + 2 * NB + 6 * n + 2


def buscar_trama(buf: bytes):
    """Devuelve (trama, resto) con la primera trama A5 5D valida, o (None, buf)."""
    i = 0
    while True:
        i = buf.find(b"\xA5\x5D", i)
        if i < 0 or len(buf) - i < ENC:
            return None, buf
        n = struct.unpack_from("<H", buf, i + 8)[0]
        if not (N <= n <= 8 * N and n % N == 0):
            i += 1
            continue
        t = tamano(n)
        if len(buf) - i < t:
            return None, buf
        trama = buf[i:i + t]
        if crc16(trama[2:-2]) == struct.unpack_from("<H", trama, t - 2)[0]:
            return trama, buf[i + t:]
        i += 1


def decodificar(trama: bytes) -> dict:
    ver, escucha, perfil, n_int = trama[2], trama[3], trama[4], trama[5]
    bloques, n, fs, blo, bhi, rlo, rhi = struct.unpack_from("<7H", trama, 6)
    q = ENC
    interf = np.frombuffer(trama, "<u2", MAX_INTERF, q)[:n_int].astype(int)
    q += 2 * MAX_INTERF
    piso_cdb = np.frombuffer(trama, "<i2", NB, q).astype(float)
    q += 2 * NB
    x, e, f = (np.frombuffer(trama, "<i2", n, q + 2 * n * j).astype(float) / 32768 for j in range(3))
    return dict(version=ver, escucha=escucha, perfil=perfil, bloques=bloques, n=n, fs=fs,
                banda=(blo, bhi), rint=(rlo, rhi), interf=interf, piso=10 ** (piso_cdb / 1000), x=x, esp=e, fir=f)


def espectro(v):
    """Promedio de periodogramas de bloques de N con Hann periodica: |X|^2 / sum(w^2), como el nodo."""
    w = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(N) / N)
    b = v[: len(v) // N * N].reshape(-1, N)
    b = b - b.mean(axis=1, keepdims=True)
    return (np.abs(np.fft.rfft(b * w, axis=1)) ** 2 / np.sum(w ** 2)).mean(axis=0)


def resumen(c):
    """Atenuacion en el rango de interferencia y energia conservada en la banda util, por metodo."""
    f = np.arange(NB) * c["fs"] / N
    Px, Pe, Pf = espectro(c["x"]), espectro(c["esp"]), espectro(c["fir"])
    rng = (f >= c["rint"][0]) & (f <= c["rint"][1])
    util = (f >= c["banda"][0]) & (f <= c["banda"][1])
    db = lambda a, b: 10 * np.log10(max(a, 1e-30) / max(b, 1e-30))
    filas = [("atenuacion en rint (dB)", db(Px[rng].sum(), Pe[rng].sum()), db(Px[rng].sum(), Pf[rng].sum())),
             ("energia en banda util conservada (%)", 100 * Pe[util].sum() / max(Px[util].sum(), 1e-30),
              100 * Pf[util].sum() / max(Px[util].sum(), 1e-30))]
    for k in c["interf"]:
        filas.append((f"atenuacion en {k * c['fs'] / N:.0f} Hz (dB)", db(Px[k], Pe[k]), db(Px[k], Pf[k])))
    return filas


def graficar(c, png, mostrar):
    import matplotlib
    if not mostrar:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(c["n"]) / c["fs"] * 1000
    f = np.arange(NB) * c["fs"] / N
    fig, ax = plt.subplots(3, 1, figsize=(11, 10))
    fig.suptitle(f"Captura del Nodo 3: {c['bloques']} bloques, parlante (GPIO25) = {ESCUCHA.get(c['escucha'], '?')}, "
                 f"FIR {PERFIL.get(c['perfil'], '?')}")
    estilos = [("x", "entrada (Nodo 1)", 1.2), ("esp", "espectral (Nodo 3)", 1.0), ("fir", "FIR (Nodo 2)", 1.0)]
    for clave, nombre, lw in estilos:
        ax[0].plot(t, c[clave], lw=lw, label=nombre, alpha=0.8)
    ax[0].set(xlabel="tiempo [ms]", ylabel="amplitud (escala completa = 1)", title="Formas de onda (alineadas)")
    ax[0].legend(loc="upper right")
    m = t <= t[0] + 30
    for clave, nombre, lw in estilos:
        ax[1].plot(t[m], c[clave][m], lw=lw, label=nombre)
    ax[1].set(xlabel="tiempo [ms]", ylabel="amplitud", title="Acercamiento: primeros 30 ms")
    ax[1].legend(loc="upper right")
    tope = -np.inf
    for clave, nombre, lw in estilos:
        p_db = 10 * np.log10(espectro(c[clave]) + 1e-20)
        tope = max(tope, p_db.max())
        ax[2].plot(f, p_db, lw=lw, label=nombre)
    ax[2].plot(f, 10 * np.log10(c["piso"] + 1e-20), "k--", lw=1, label="piso de ruido usado por el nodo")
    ax[2].axvspan(*c["banda"], color="g", alpha=0.12, label="banda util")
    ax[2].axvspan(*c["rint"], color="r", alpha=0.10, label="rango de interferencia")
    for k in c["interf"]:
        ax[2].axvline(k * c["fs"] / N, color="r", ls=":", lw=1)
    ax[2].set(xlabel="frecuencia [Hz]", ylabel="PSD [dB]", title="Espectros (bloques de 1024, Hann)", xlim=(0, c["fs"] / 2), ylim=(tope - 100, tope + 10))
    ax[2].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    print(f"Figura guardada en {png}")
    if mostrar:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("puerto", nargs="?")
    ap.add_argument("--bloques", type=int, default=4, help="1 a 8 bloques de 1024 (64 ms cada uno)")
    ap.add_argument("--archivo", help="grafica una captura guardada en vez de pedirla a la placa")
    ap.add_argument("--sin-mostrar", action="store_true", help="solo guarda la figura")
    a = ap.parse_args()
    CARPETA.mkdir(exist_ok=True)
    if a.archivo:
        trama, _ = buscar_trama(Path(a.archivo).read_bytes())
        if trama is None:
            sys.exit("el archivo no contiene una captura valida")
        base = Path(a.archivo).with_suffix("")
    else:
        if not a.puerto:
            ap.error("indique el puerto (p. ej. COM7) o --archivo")
        import serial
        ser = serial.Serial(a.puerto, 115200, timeout=0.2)
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(f"captura {a.bloques}\n".encode())
        buf, trama, limite = b"", None, time.time() + 25
        while trama is None and time.time() < limite:
            buf += ser.read(4096)
            trama, buf = buscar_trama(buf)
        ser.close()
        if trama is None:
            sys.exit("no llego una captura valida (el nodo 3 esta procesando bloques? 'src uart' con el nodo 1 o 'src gen')")
        base = CARPETA / f"captura_{datetime.now():%Y%m%d_%H%M%S}"
        base.with_suffix(".bin").write_bytes(trama)
        print(f"Captura guardada en {base.with_suffix('.bin')}")
    c = decodificar(trama)
    print(f"{c['bloques']} bloques, banda util {c['banda'][0]}-{c['banda'][1]} Hz, rango de interferencia "
          f"{c['rint'][0]}-{c['rint'][1]} Hz, interferentes: {[round(k * c['fs'] / N) for k in c['interf']]} Hz")
    print(f"{'':40s}{'espectral (N3)':>16s}{'FIR (N2)':>12s}")
    for nombre, e, fi in resumen(c):
        print(f"{nombre:40s}{e:16.1f}{fi:12.1f}")
    graficar(c, base.with_suffix(".png"), not a.sin_mostrar)


if __name__ == "__main__":
    main()
