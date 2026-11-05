"""Prueba 2: recepcion UART (parser de tramas) contra las tramas del propio proyecto en Python.

Genera flujos con link.empaquetar() (PCM) y receptor_fft.empaquetar() (parametros del Nodo 1),
los degrada (perdidas, errores de bit, basura, fragmentacion) y exige que el parser C++ acepte
exactamente las tramas validas y ninguna danada. Uso (desde Proyecto_2/):
    python nodo3_esp32/tests/test_link.py
"""
import os, subprocess, sys, tempfile, struct
from pathlib import Path
import numpy as np

RAIZ = Path(__file__).resolve().parents[2]
FW = RAIZ / "nodo3_esp32"
sys.path.insert(0, str(RAIZ)); sys.path.insert(0, str(RAIZ / "full_system"))
os.chdir(RAIZ.parent)
from sistema_ruido import link                                  # noqa: E402
from receptor_fft.src import receptor_fft as rf                 # noqa: E402

TMP = Path(tempfile.mkdtemp(prefix="nodo3_link_"))
BIN = TMP / "host_link"
fallos = []


def chequeo(nombre, ok, detalle=""):
    print(f"[{'OK ' if ok else 'FALLA'}] {nombre}: {detalle}")
    if not ok:
        fallos.append(nombre)


def fnv(m):
    h = 2166136261
    for v in m.astype("<i2").view("<u2"):
        h = ((h ^ int(v)) * 16777619) & 0xFFFFFFFF
    return f"{h:08x}"


def correr(flujo, trozo=None):
    f = TMP / "flujo.bin"; f.write_bytes(flujo)
    args = [str(BIN), str(f)] + ([str(trozo)] if trozo else [])
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout.splitlines()
    ev = [l for l in out if not l.startswith("STATS")]
    st = dict(kv.split("=") for kv in out[-1].split()[1:])
    return ev, {k: int(v) for k, v in st.items()}


def main():
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(BIN),
                    str(FW / "tests/host_link.cpp"), str(FW / "communication.cpp")], check=True)
    rng = np.random.default_rng(7)
    out = subprocess.run([str(BIN), "crc", "123456789"], capture_output=True, text=True).stdout.strip()
    chequeo("CRC-16 C++ == 0x29B1 (123456789)", out == "29b1" and link.crc16(b"123456789") == 0x29B1, out)

    bloques = rng.integers(-30000, 30000, (40, 1024)).astype("<i2")
    bloques[3, :] = 0x0201   # patrones que imitan SYNC/tipo dentro de los datos
    bloques[4, 100:103] = np.array([0x5AA5, 0x0002, 0x0400], dtype=np.uint16).view("<i2")   # bytes A5 5A 02 00 00 04
    pcm = [link.empaquetar(i, b) for i, b in enumerate(bloques)]
    frec = [dict(f_hz=2500.0 + 0.25 * i, snr_local_db=30.0 - i, flags=1) for i in range(3)]
    par = rf.empaquetar(9, frec)
    vacio = rf.empaquetar(10, [])

    # 1. flujo limpio mezclado, con cualquier fragmentacion
    flujo = b"".join(p + (par if i % 2 else vacio) for i, p in enumerate(pcm))
    esperado = [f"PCM {i} {fnv(bloques[i])}" for i in range(40)]
    for trozo in (None, 1, 7, 100, 4096):
        ev, st = correr(flujo, trozo)
        pc = [e for e in ev if e.startswith("PCM")]
        chequeo(f"limpio, trozos de {trozo or 'todo'} B", pc == esperado and st["crc_err"] == 0 and st["descartados"] == 0
                and st["params"] == 40, f"{len(pc)}/40 PCM, {st['params']}/40 PARAMS, crc_err={st['crc_err']}, descartados={st['descartados']}")
    ev, _ = correr(flujo)
    p1 = [e for e in ev if e.startswith("PARAMS 9")][0]
    _, dec = rf.desempaquetar(par)
    chequeo("PARAMS decodificados coinciden con el Python del Nodo 1",
            all(abs(float(x.split(",")[0]) - d["f_hz"]) < 1e-9 for x, d in zip(p1.split()[3:], dec)), p1)

    # 2. trama de parametros vacia cuyo CRC bajo vale 0x04 (ambigua con n=1024 de una PCM)
    for sec in range(256):
        t = rf.empaquetar(sec, [])
        if t[5] == 0x04 and t[2] == 0x02:
            ev, st = correr(t + pcm[0] + t + pcm[1])
            ok = [e for e in ev if e.startswith("PARAMS")] == [f"PARAMS {sec} 0"] * 2 and st["pcm"] == 2
            chequeo(f"PARAMS vacio con crc_lo=0x04 (sec={sec}) desambiguado", ok, f"params={st['params']} pcm={st['pcm']}")
            break

    # 3. basura, trama truncada y recuperacion
    basura = bytes(rng.integers(0, 256, 3000, dtype=np.uint8))
    trunc = pcm[5][:1500]
    flujo = basura + pcm[0] + trunc + pcm[1] + b"\xA5" * 7 + pcm[2] + b"\xA5\x5A" + pcm[3]
    ev, st = correr(flujo)
    seqs = [int(e.split()[1]) for e in ev if e.startswith("PCM")]
    chequeo("basura/truncada: se recupera y entrega solo tramas validas", seqs[:1] == [0] and 3 in seqs and 2 in seqs
            and all(fnv(bloques[s]) in " ".join(ev) for s in seqs), f"seqs recibidas {seqs}; crc_err={st['crc_err']}")

    # 4. errores de bit + perdidas: debe coincidir con link.desempaquetar_flujo (la referencia del proyecto)
    for ber, ploss in ((1e-5, 0.0), (1e-4, 0.1), (5e-4, 0.2)):
        perdidas = {i for i in range(40) if rng.random() < ploss}
        raw = np.frombuffer(b"".join(p for i, p in enumerate(pcm) if i not in perdidas), dtype=np.uint8)
        bits = np.unpackbits(raw); bits ^= (rng.random(bits.size) < ber).astype(np.uint8)
        flujo = np.packbits(bits).tobytes()
        ev, st = correr(flujo, 333)
        rx = {int(e.split()[1]): e.split()[2] for e in ev if e.startswith("PCM")}
        ref = link.desempaquetar_flujo(flujo)
        mismos = set(rx) == set(ref) and all(rx[s] == fnv(ref[s]) for s in rx)
        falsas = [s for s in rx if s >= 40 or rx[s] != fnv(bloques[s])]
        chequeo(f"BER={ber:g}, perdida={ploss:.0%}", mismos and not falsas,
                f"C++ acepta {len(rx)} == Python {len(ref)}; tramas danadas aceptadas: {len(falsas)}; crc_err={st['crc_err']}")

    # 5. numeracion: perdidas, repetidas y envoltura u16
    s = subprocess.run([str(BIN), "seq", "65536", "65534", "65535", "0", "1", "4", "4", "3", "5"],
                       capture_output=True, text=True).stdout.strip()
    chequeo("secuencia: huecos, repetidas y envoltura u16", s == "0 0 0 0 2 -1 -1 0 | perdidas=2 repetidas=2", s)

    # 5b. formato de entrada: cuenta cruda del ADC (firmware actual del Nodo 1) y Q15
    r = [float(subprocess.run([str(BIN), "fmt", f, "2048", str(m)], capture_output=True, text=True).stdout)
         for f, m in (("adc12", 3072), ("adc12", 2048), ("adc12", 0), ("adc12", 4095), ("q15", 16384), ("q15", -32768))]
    chequeo("formato de entrada adc12 / q15", np.allclose(r, [0.5, 0.0, -1.0, 2047 / 2048, 0.5, -1.0], atol=1e-6), str(r))

    # 6. ancho de banda (justificacion de la velocidad)
    d = link.disenar_enlace()
    u921 = d["bits_s"] / 921600
    chequeo("enlace: 460 800 baud es el minimo de link.disenar_enlace; el Nodo 1 usa 921 600", d["baud"] == 460800, 
            f"trama {d['bytes_trama']} B, {d['bits_s']/1e3:.1f} kb/s netos, utilizacion {100*d['utilizacion']:.1f} % a 460 800 y {100*u921:.1f} % a 921 600")
    print("\nRESULTADO:", "TODAS LAS COMPROBACIONES PASARON" if not fallos else f"FALLARON: {fallos}")
    sys.exit(1 if fallos else 0)


main()
