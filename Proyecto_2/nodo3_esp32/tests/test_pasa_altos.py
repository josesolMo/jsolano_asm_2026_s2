"""Prueba en el PC del pasa altos de entrada (pasa_altos.h): respuesta medida con tonos y DC."""
from pathlib import Path
import subprocess
import sys
import tempfile

FW = Path(__file__).resolve().parents[1]
PROG = r'''
#define NODO3_HOST 1
#include "pasa_altos.h"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace n3;
int main() {
  const float fs[] = {0, 50, 60, 100, 150, 300, 1000, 2500, 7000};
  for (float f : fs) {
    PasaAltos h; h.configurar(100);
    std::vector<float> x(32768);
    for (size_t i = 0; i < x.size(); ++i) x[i] = f == 0 ? 0.5f : 0.5f * std::sin(2 * M_PI * f * i / 16000.0);
    for (size_t i = 0; i < x.size(); i += 1024) h.procesar(&x[i], 1024);
    double e = 0; for (size_t i = 16768; i < x.size(); ++i) e += x[i] * x[i];
    const double ref = f == 0 ? 0.25 : 0.125;
    printf("%.0f %.2f\n", f, 10 * std::log10(e / 16000 / ref + 1e-30));
  }
}
'''


def main():
    tmp = Path(tempfile.mkdtemp())
    (tmp / "p.cpp").write_text(PROG)
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", str(FW), "-o", str(tmp / "p"), str(tmp / "p.cpp")],
                   check=True)
    r = dict(tuple(map(float, l.split())) for l in subprocess.run([str(tmp / "p")], capture_output=True, text=True,
                                                                   check=True).stdout.split("\n") if l)
    for f, g in r.items():
        print(f"{f:6.0f} Hz: {g:7.2f} dB")
    ok = (r[0] < -60 and r[50] < -11 and r[60] < -8 and abs(r[100] + 3.01) < 0.2 and -1.5 < r[150] < -0.5
          and r[300] > -0.1 and abs(r[1000]) < 0.01 and abs(r[7000]) < 0.01)
    print("RESULTADO PASA ALTOS:", "OK" if ok else "FALLA")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
