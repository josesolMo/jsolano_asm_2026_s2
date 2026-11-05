// Harness de 'recomendar': lee el flujo UART del Nodo 1 (enlace_host), lo pasa por el parser y
// aplica recomendar() en cada bloque con el ultimo PARAMS/PISO. Imprime cuantos bloques recomiendan
// cada modo y la explicacion del ultimo.
// Uso: host_recomendar flujo.bin banda_lo banda_hi perfil(0..3)
#ifndef NODO3_HOST
#define NODO3_HOST 1
#endif
#include "../communication.h"
#include "../spectral_filter.h"
#include "../recomendacion.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
using namespace n3;

struct Sink : FrameParser::Sink {
  ParamsFrame par{}; bool hay_par = false;
  Diag diag; int bloques = 0, fir = 0, filtro = 0;
  float blo = 0, bhi = 0; PerfilFir perfil = PerfilFir::Tono;
  char texto[2048];
  void on_pcm(uint16_t, const int16_t*) override {}
  void on_params(const ParamsFrame& f) override { par = f; hay_par = true; }
  void on_piso(const PisoFrame& f) override {
    diag_set_piso_knots(diag, f.centi_db, f.nk, f.decim);
    if (!hay_par || ++bloques <= 8) return;          // la EMA del piso necesita unos bloques
    float fz[MAX_FREQ]; int8_t sn[MAX_FREQ]; uint8_t fl[MAX_FREQ];
    for (int i = 0; i < par.n; ++i) { fz[i] = par.f[i].f_hz; sn[i] = par.f[i].snr_db; fl[i] = par.f[i].flags; }
    const Recomendacion r = recomendar(fz, sn, fl, par.n, diag.piso, diag.piso_valido, blo, bhi, perfil, G_INTERF,
                                       texto, sizeof texto);
    (r.modo == Modo::Fir ? fir : filtro)++;
  }
};

int main(int argc, char** argv) {
  if (argc < 5) return 1;
  FILE* f = fopen(argv[1], "rb"); if (!f) return 2;
  std::vector<uint8_t> b; int c; while ((c = fgetc(f)) != EOF) b.push_back(uint8_t(c));
  fclose(f);
  static Sink s; s.diag.piso_valido = false;
  s.blo = float(atof(argv[2])); s.bhi = float(atof(argv[3])); s.perfil = PerfilFir(atoi(argv[4]));
  FrameParser p; p.feed(b.data(), b.size(), s);
  printf("%s", s.texto);
  printf("CONTEO fir=%d filter=%d\n", s.fir, s.filtro);
  return 0;
}
