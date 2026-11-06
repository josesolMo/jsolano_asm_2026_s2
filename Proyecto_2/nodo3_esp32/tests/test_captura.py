"""Prueba en el PC del comando 'captura': captura.h (firmware) arma la trama y
tools/graficar_captura.py la encuentra entre texto, la decodifica, resume y grafica."""
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
import graficar_captura as gc   # noqa: E402


def main():
    tmp = Path(tempfile.mkdtemp())
    exe = tmp / "host_captura"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(exe),
                    str(FW / "tests/host_captura.cpp"), str(FW / "communication.cpp")], check=True)
    subprocess.run([str(exe), str(tmp / "c.bin"), "2"], check=True)
    trama, _ = gc.buscar_trama((tmp / "c.bin").read_bytes())
    ok = trama is not None
    if ok:
        c = gc.decodificar(trama)
        t = np.arange(c["n"]) / 16000
        x = 0.4 * np.sin(2 * np.pi * 1000 * t) + 0.3 * np.sin(2 * np.pi * 2500 * t)
        filas = dict((a, (b, d)) for a, b, d in gc.resumen(c))
        ok = (c["n"] == 2048 and c["banda"] == (900, 1100) and list(c["interf"]) == [160]
              and np.max(np.abs(c["x"] - x)) < 1e-4 and abs(10 * np.log10(c["piso"][0]) + 70) < 0.01
              and filas["atenuacion en rint (dB)"][0] > 50 and 30 < filas["atenuacion en rint (dB)"][1] < 50
              and 95 < filas["energia en banda util conservada (%)"][1] < 97)
        gc.graficar(c, tmp / "c.png", False)
        ok = ok and (tmp / "c.png").stat().st_size > 10000
        print("resumen:", {k: tuple(round(v, 1) for v in vs) for k, vs in filas.items()})
    print("RESULTADO CAPTURA:", "OK" if ok else "FALLA")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
