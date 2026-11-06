// Harness de la salida FIR (salida_fir.h): lee float32, procesa por avances de HOP y escribe
// la salida (float32). Imprime la atenuacion medida por StatsSenales en el rango dado.
// Uso: host_fir perfil(0..3) entrada.bin salida.bin rint_lo rint_hi
#ifndef NODO3_HOST
#define NODO3_HOST 1
#endif
#include "../salida_fir.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace n3;

int main(int argc, char** argv) {
  if (argc < 6) return 1;
  static SalidaFir fir; static StatsSenales st;
  if (!st.begin()) return 2;
  fir.set_perfil(PerfilFir(atoi(argv[1])));
  FILE* f = fopen(argv[2], "rb"); std::vector<float> x; float v;
  while (fread(&v, 4, 1, f) == 1) x.push_back(v);
  fclose(f);
  Params p; p.rint_lo = float(atof(argv[4])); p.rint_hi = float(atof(argv[5]));
  std::vector<float> y(x.size()); std::vector<float> prev(HOP, 0.0f);
  double rin = 0, rout = 0;
  for (size_t i = 0; i + HOP <= x.size(); i += HOP) {
    fir.procesar(&x[i], &y[i]);
    HopStats h; st.on_hop(prev.data(), &y[i], nullptr, p, &h);
    if (i >= 4 * HOP) { rin += h.p_rng_in; rout += h.p_rng_out; }
    for (int k = 0; k < HOP; ++k) prev[k] = x[i + k];
  }
  FILE* o = fopen(argv[3], "wb"); fwrite(y.data(), 4, y.size(), o); fclose(o);
  printf("retardo_fir=%d atenuacion_rango_db=%.3f\n", fir.retardo(), 10.0 * log10(rin / rout));
  return 0;
}
