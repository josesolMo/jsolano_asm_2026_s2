"""Prueba 2: recepcion de tramas (parser) contra las tramas reales del Nodo 1 y del proyecto.

Usa el codigo del propio Nodo 1 (nodo1_esp32s3/trama.h) y link.empaquetar para generar flujos, los degrada
(perdidas, errores de bit, basura, fragmentacion) y exige que el parser C++ del Nodo 3 acepte exactamente las
tramas validas y ninguna danada. Es una prueba de escritorio: no usa UART ni SPI reales.
Uso (desde la raiz del repositorio):  python Proyecto_2/nodo3_espectral/tests/test_link.py
"""
import os, subprocess, sys, tempfile
from pathlib import Path
import numpy as np

RAIZ = Path(__file__).resolve().parents[2]            # Proyecto_2/
FW = RAIZ / "nodo3_espectral"
sys.path.insert(0, str(RAIZ)); sys.path.insert(0, str(RAIZ / "full_system"))
os.chdir(RAIZ.parent)                                  # config.py crea un directorio relativo a la raiz del repositorio
from sistema_ruido import link                         # noqa: E402

TMP = Path(tempfile.mkdtemp(prefix="nodo3_link_"))
BIN, GEN = TMP / "host_link", TMP / "gen_nodo1"
fallos = []


def chequeo(nombre, ok, detalle=""):
    print(f"[{'OK ' if ok else 'FALLA'}] {nombre}: {detalle}")
    if not ok:
        fallos.append(nombre)


def fnv(m):
    h = 2166136261
    for v in np.asarray(m).astype("<i2").view("<u2"):
        h = ((h ^ int(v)) * 16777619) & 0xFFFFFFFF
    return f"{h:08x}"


def correr(flujo, trozo=None):
    f = TMP / "flujo.bin"; f.write_bytes(flujo)
    out = subprocess.run([str(BIN), str(f)] + ([str(trozo)] if trozo else []), check=True,
                         capture_output=True, text=True).stdout.splitlines()
    st = {k: int(v) for k, v in (kv.split("=") for kv in out[-1].split()[1:])}
    return out[:-1], st


def main():
    for exe, srcs in ((BIN, ["host_link.cpp", "../communication.cpp"]), (GEN, ["gen_nodo1.cpp"])):
        subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(exe)]
                       + [str(FW / "tests" / s) for s in srcs], check=True)
    rng = np.random.default_rng(7)
    out = subprocess.run([str(BIN), "crc", "123456789"], capture_output=True, text=True).stdout.strip()
    chequeo("CRC-16 C++ == 0x29B1 (123456789)", out == "29b1" and link.crc16(b"123456789") == 0x29B1, out)

    # 1. flujo generado por el codigo REAL del Nodo 1 (trama.h): muestras + diagnostico + espectro a peticion
    subprocess.run([str(GEN), str(TMP / "n1.bin"), str(TMP / "n1.txt"), "12"], check=True)
    flujo, esperado = (TMP / "n1.bin").read_bytes(), (TMP / "n1.txt").read_text().splitlines()
    for trozo in (None, 1, 7, 100, 4096):
        ev, st = correr(flujo, trozo)
        chequeo(f"trama.h del Nodo 1, trozos de {trozo or 'todo'} B", ev == esperado and st["crc_err"] == 0,
                f"{st['pcm']}/12 PCM, {st['diag']}/24 DIAG idénticas a las enviadas, crc_err={st['crc_err']}, "
                f"bytes ignorados={st['descartados']} (trama de espectro 5C)")
    chequeo("la trama de espectro (5C) se ignora sin perder las muestras siguientes",
            sum(e.startswith("PCM") for e in ev) == 12, "12/12 PCM")

    # 2. PCM de link.py == PCM de trama.h (mismo formato) y el parser acepta ambas
    bloques = rng.integers(0, 4096, (40, 1024)).astype("<i2")
    pcm = [link.empaquetar(i, b) for i, b in enumerate(bloques)]
    ev, st = correr(b"".join(pcm), 333)
    chequeo("PCM de link.empaquetar", [e for e in ev if e.startswith("PCM")] == [f"PCM {i} {fnv(bloques[i])}" for i in range(40)],
            f"{st['pcm']}/40")

    # 3. basura, trama truncada y recuperacion
    basura = bytes(rng.integers(0, 256, 3000, dtype=np.uint8))
    flujo = basura + pcm[0] + pcm[5][:1500] + pcm[1] + b"\xA5" * 7 + pcm[2] + b"\xA5\x5A" + pcm[3]
    ev, st = correr(flujo)
    seqs = [int(e.split()[1]) for e in ev if e.startswith("PCM")]
    chequeo("basura/truncada: se recupera y entrega solo tramas validas", seqs[:1] == [0] and 3 in seqs and 2 in seqs
            and all(fnv(bloques[s]) in " ".join(ev) for s in seqs), f"seqs recibidas {seqs}; crc_err={st['crc_err']}")

    # 4. errores de bit + perdidas, con diagnosticos intercalados: mismas tramas PCM que link.desempaquetar_flujo
    mezcla = []
    subprocess.run([str(GEN), str(TMP / "m.bin"), str(TMP / "m.txt"), "40"], check=True)
    for ber, ploss in ((1e-5, 0.0), (1e-4, 0.1), (5e-4, 0.2)):
        base = (TMP / "m.bin").read_bytes()
        raw = np.frombuffer(base, dtype=np.uint8)
        bits = np.unpackbits(raw); bits ^= (rng.random(bits.size) < ber).astype(np.uint8)
        flujo = np.packbits(bits).tobytes()
        ev, st = correr(flujo, 333)
        rx = {int(e.split()[1]): e.split()[2] for e in ev if e.startswith("PCM")}
        ref = link.desempaquetar_flujo(flujo)
        orig = {l.split()[1]: l.split()[2] for l in (TMP / "m.txt").read_text().splitlines() if l.startswith("PCM")}
        mismos = set(rx) == set(ref) and all(rx[s] == fnv(ref[s]) for s in rx)
        falsas = [s for s in rx if orig[str(s)] != rx[s]]
        chequeo(f"BER={ber:g}", mismos and not falsas,
                f"C++ acepta {len(rx)} PCM == Python {len(ref)}; PCM dañadas aceptadas: {len(falsas)}; crc_err={st['crc_err']}")

    # 5. trama de piso propuesta (0x5D): ida y vuelta y rechazo de corrupcion
    cdb = [-9000 + 37 * i for i in range(65)]
    subprocess.run([str(BIN), "build_piso", str(TMP / "piso.bin"), "77", "8", *map(str, cdb)], check=True)
    pb = (TMP / "piso.bin").read_bytes()
    ev, _ = correr(pb + pcm[0] + pb)
    chequeo("trama PISO (5D): 65 nudos", ev[0] == "PISO 77 8 65 " + " ".join(map(str, cdb)) and len(pb) == 140, f"{len(pb)} B")
    dañada = bytearray(pb); dañada[20] ^= 1
    ev, st = correr(bytes(dañada) + pcm[1])
    chequeo("PISO con un bit dañado se rechaza", not any(e.startswith("PISO") for e in ev) and st["crc_err"] >= 1 and
            any(e.startswith("PCM 1") for e in ev), f"crc_err={st['crc_err']}, PCM siguiente recuperada")

    # 6. numeracion, formato de entrada y ancho de banda
    s = subprocess.run([str(BIN), "seq", "65536", "65534", "65535", "0", "1", "4", "4", "3", "5"],
                       capture_output=True, text=True).stdout.strip()
    chequeo("secuencia: huecos, repetidas y envoltura u16", s == "0 0 0 0 2 -1 -1 0 | perdidas=2 repetidas=2", s)
    r = [float(subprocess.run([str(BIN), "fmt", f, "2048", str(m)], capture_output=True, text=True).stdout)
         for f, m in (("adc12", 3072), ("adc12", 2048), ("adc12", 0), ("adc12", 4095), ("q15", 16384), ("q15", -32768))]
    chequeo("formato de entrada adc12 / q15", np.allclose(r, [0.5, 0.0, -1.0, 2047 / 2048, 0.5, -1.0], atol=1e-6), str(r))
    d = link.disenar_enlace()
    chequeo("enlace: 460 800 baud es el minimo de link.disenar_enlace; el Nodo 1 usa 921 600", d["baud"] == 460800,
            f"trama {d['bytes_trama']} B, {d['bits_s']/1e3:.1f} kb/s netos, utilizacion {100*d['utilizacion']:.1f} % a 460 800 "
            f"y {100*d['bits_s']/921600:.1f} % a 921 600")
    print("\nRESULTADO:", "TODAS LAS COMPROBACIONES PASARON" if not fallos else f"FALLARON: {fallos}")
    sys.exit(1 if fallos else 0)


main()
