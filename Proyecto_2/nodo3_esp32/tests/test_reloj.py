"""Prueba en el PC de la compensacion de reloj del DAC (audio_output.h).

Parte 1, ControlReloj (modo 'reloj apll', el de defecto): el DAC se acelera o frena reprogramando su
APLL. Se simula con la resolucion real del APLL (~1 ppm) y el umbral de 100 ppm.
Parte 2, AjusteReloj (modo 'reloj interp', respaldo): remuestreo lineal.

Simula el Nodo 1 muestreando mas rapido o mas lento que el DAC (medido en la placa: ~+0.36 %),
con el DAC consumiendo tramos de 256 muestras como el DMA. Comprueba que el anillo no se desborda
ni se vacia, que el ajuste converge a la diferencia de relojes y que un tono de 1 kHz sale sin
distorsion apreciable (lo que no es el tono queda > 40 dB abajo).
"""
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

FW = Path(__file__).resolve().parents[1]
PROGRAMA = r'''
#define NODO3_HOST 1
#include "audio_output.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace n3;
int main(int, char** argv) {
  const double rel = atof(argv[1]);          // fs del Nodo 1 / fs del DAC
  AjusteReloj aj; std::vector<float> anillo, salida;
  double t_in = 0, t_dac = 0; const double dt = 512.0 / 16000.0 / rel;  // una llamada por avance (512 muestras)
  size_t maxn = 0, minn = 1u << 30; long fase = 0; float out[700];
  for (int it = 0; it < 3000; ++it) {
    float x[512];
    for (int i = 0; i < 512; ++i, ++fase) x[i] = 0.5f * std::sin(2 * M_PI * 1000.0 * fase / (16000.0 * rel));
    const size_t k = aj.procesar(x, 512, float(anillo.size()), out);
    anillo.insert(anillo.end(), out, out + k);
    t_in += dt;
    while (t_dac + 256.0 / 16000.0 <= t_in) {   // el DMA saca tramos de 256 al ritmo del DAC
      t_dac += 256.0 / 16000.0;
      if (anillo.size() >= 256) { salida.insert(salida.end(), anillo.begin(), anillo.begin() + 256); anillo.erase(anillo.begin(), anillo.begin() + 256); }
      else { printf("VACIO\n"); anillo.clear(); }
    }
    if (it > 500) { maxn = anillo.size() > maxn ? anillo.size() : maxn; minn = anillo.size() < minn ? anillo.size() : minn; }
  }
  printf("ANILLO %zu %zu PPM %.0f\n", minn, maxn, aj.ppm());
  FILE* f = fopen(argv[2], "wb"); fwrite(salida.data(), 4, salida.size(), f); fclose(f);
}
'''


APLL = r'''
#define NODO3_HOST 1
#include "audio_output.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace n3;
int main(int, char** argv) {
  const double rel = atof(argv[1]);
  ControlReloj c; double nivel = 1024, e_apl = 0; int cambios = 0; double mn = 1e9, mx = 0;
  for (int it = 0; it < 30 * 31; ++it) {                 // 30 s, una llamada cada 32 ms (512 muestras)
    nivel += 512.0 - 512.0 * (1.0 + e_apl) / rel;        // entra 512 del Nodo 1; el DAC saca a su ritmo
    const double ruido = (it % 2) ? 128 : -128;           // el DMA saca de a 256: el nivel salta
    const float e = c.actualizar(float(nivel + ruido));
    if (std::fabs(e - e_apl) >= ControlReloj::UMBRAL) { e_apl = std::round(e * 1e6) / 1e6; ++cambios; }
    if (it > 20 * 31) { mn = nivel < mn ? nivel : mn; mx = nivel > mx ? nivel : mx; }
  }
  printf("APLL %.0f %.0f %.0f %d\n", mn, mx, e_apl * 1e6, cambios);
}
'''


def main():
    tmp = Path(tempfile.mkdtemp())
    (tmp / "a.cpp").write_text(APLL)
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", str(FW), "-o", str(tmp / "a"),
                    str(tmp / "a.cpp")], check=True)
    fallas_apll = 0
    for rel in [1.0036, 0.9964, 1.0]:
        mn, mx, ppm, cambios = subprocess.run([str(tmp / "a"), str(rel)], check=True, capture_output=True,
                                              text=True).stdout.split()[1:]
        mn, mx, ppm, cambios = float(mn), float(mx), float(ppm), int(cambios)
        ok = 256 < mn and mx < 4096 - 512 and abs(ppm - (rel - 1) * 1e6) < 200
        fallas_apll += not ok
        print(f"APLL, relojes {100 * (rel - 1):+.2f} %: anillo {mn:.0f}..{mx:.0f} (ultimos 10 s), ajuste {ppm:+.0f} ppm, "
              f"{cambios} reprogramaciones en 30 s {'OK' if ok else 'FALLA'}")
    (tmp / "p.cpp").write_text(PROGRAMA)
    exe = tmp / "p"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", str(FW), "-o", str(exe), str(tmp / "p.cpp")],
                   check=True)
    fallas = 0
    for rel in [1.0036, 0.9964, 1.0]:
        out = subprocess.run([str(exe), str(rel), str(tmp / "y.bin")], check=True, capture_output=True, text=True).stdout
        mn, mx, ppm = out.split("ANILLO ")[1].split()[0], out.split()[2], float(out.split("PPM ")[1])
        mn, mx = int(mn), int(mx)
        y = np.fromfile(tmp / "y.bin", dtype=np.float32)[-16000 * 10:]
        Y = np.abs(np.fft.rfft(y * np.hanning(y.size))) ** 2
        f = np.fft.rfftfreq(y.size, 1 / 16000)
        tono = (f > 990) & (f < 1010)
        resto_db = 10 * np.log10(Y[~tono].sum() / Y[tono].sum())
        ok = "VACIO" not in out and mx < 4096 - 512 and mn > 0 and abs(ppm - (rel - 1) * 1e6) < 600 and resto_db < -40
        fallas += not ok
        print(f"Interp, relojes {100 * (rel - 1):+.2f} %: anillo {mn}..{mx} de 4096, ajuste {ppm:+.0f} ppm, "
              f"todo lo que no es el tono {resto_db:.1f} dB {'OK' if ok else 'FALLA'}")
    fallas += fallas_apll
    print("RESULTADO RELOJ:", "OK" if not fallas else "FALLA")
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
