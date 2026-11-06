// Harness de captura.h: arma una trama con senales conocidas (como en procesar_float, de a HOP).
// Uso: host_captura salida.bin bloques
#ifndef NODO3_HOST
#define NODO3_HOST 1
#endif
#include "../captura.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace n3;

int main(int argc, char** argv) {
  if (argc < 3) return 1;
  const size_t n = size_t(atoi(argv[2])) * N;
  std::vector<uint8_t> buf(captura_tamano(n));
  float x[HOP], e[HOP], f[HOP];
  for (size_t i0 = 0; i0 < n; i0 += HOP) {
    for (int i = 0; i < HOP; ++i) {
      const double t = double(i0 + i) / FS;
      const float u = float(0.4 * sin(2 * M_PI * 1000 * t)), r = float(0.3 * sin(2 * M_PI * 2500 * t));
      x[i] = u + r; e[i] = u + 0.001f * r; f[i] = 0.98f * u + 0.01f * r;
    }
    captura_muestras(buf.data(), n, i0, x, e, f, HOP);
  }
  static Diag d;
  for (int k = 0; k < NB; ++k) d.piso[k] = 1e-7f;
  d.piso_valido = true; d.n_interf = 1; d.interf[0] = 160;     // 2500 Hz
  Params p; p.banda_lo = 900; p.banda_hi = 1100; p.rint_lo = 2485; p.rint_hi = 2515;
  const size_t t = captura_cerrar(buf.data(), n, uint8_t(Modo::Filtro), 0, d, p);
  FILE* o = fopen(argv[1], "wb");
  fputs("#captura 2 bloques (128 ms)\nbasura de texto A5 sin trama\n", o);   // texto previo, como en el puerto
  fwrite(buf.data(), 1, t, o);
  fclose(o);
  return 0;
}
