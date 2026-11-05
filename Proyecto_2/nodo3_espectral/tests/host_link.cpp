// Harness del parser de tramas: lee un flujo de bytes, lo entrega en trozos de tamano variable
// y escribe una linea por evento. Uso: host_link flujo.bin [trozo] | host_link crc TEXTO |
//   host_link seq MOD s1 s2 ... | host_link fmt adc12|q15 mid muestra
#ifndef NODO3_HOST
#define NODO3_HOST 1
#endif
#include "../communication.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <algorithm>
using namespace n3;

struct Mostrar : FrameParser::Sink {
  void on_pcm(uint16_t seq, const int16_t* m) override {
    uint32_t h = 2166136261u;                       // hash FNV-1a de las muestras
    for (int i = 0; i < N; ++i) { h ^= uint16_t(m[i]); h *= 16777619u; }
    printf("PCM %u %08x\n", seq, h);
  }
  void on_diag(const DiagFrame& f) override {
    printf("DIAG %u %u %u", f.analisis, f.t_us, f.n);
    for (int i = 0; i < f.n; ++i) printf(" %.2f,%.4f,%.1f,%u", f.f[i].f_hz, f.f[i].amplitud, f.f[i].snr_db, f.f[i].flags);
    printf("\n");
  }
  void on_piso(const PisoFrame& f) override {
    printf("PISO %u %u %u", f.analisis, f.decim, f.nk);
    for (int i = 0; i < f.nk; ++i) printf(" %d", f.centi_db[i]);
    printf("\n");
  }
};

int main(int argc, char** argv) {
  if (argc < 2) return 1;
  const std::string cmd = argv[1];
  if (argc > 2 && cmd == "crc") { printf("%04x\n", crc16((const uint8_t*)argv[2], strlen(argv[2]))); return 0; }
  if (argc > 4 && cmd == "fmt") {
    printf("%.6f\n", muestra_a_float(int16_t(atoi(argv[4])), std::string(argv[2]) == "q15" ? FormatoIn::Q15 : FormatoIn::Adc12, atof(argv[3])));
    return 0;
  }
  if (argc > 3 && cmd == "seq") {
    SeqTracker t; const uint32_t mod = atoi(argv[2]);
    for (int i = 3; i < argc; ++i) printf("%d ", t.on_seq(atoi(argv[i]), mod));
    printf("| perdidas=%u repetidas=%u\n", t.perdidas, t.repetidas); return 0;
  }
  if (argc > 2 && cmd == "build_piso") {            // host_link build_piso salida.bin analisis decim c0 c1 ...
    uint8_t buf[300]; int16_t c[PISO_MAX_KNOTS]; const int nk = argc - 5;
    for (int i = 0; i < nk; ++i) c[i] = int16_t(atoi(argv[5 + i]));
    const size_t l = build_piso(buf, atoi(argv[3]), atoi(argv[4]), c, nk);
    FILE* f = fopen(argv[2], "wb"); fwrite(buf, 1, l, f); fclose(f); return 0;
  }
  FILE* f = fopen(argv[1], "rb"); if (!f) return 2;
  std::vector<uint8_t> b; int c; while ((c = fgetc(f)) != EOF) b.push_back(uint8_t(c));
  fclose(f);
  const size_t trozo = argc > 2 ? size_t(atoi(argv[2])) : b.size();
  FrameParser p; Mostrar s;
  for (size_t i = 0; i < b.size(); i += trozo) p.feed(b.data() + i, std::min(trozo, b.size() - i), s);
  const LinkStats& st = p.stats();
  printf("STATS bytes=%u pcm=%u diag=%u piso=%u crc_err=%u descartados=%u\n", st.bytes, st.pcm_ok,
         st.diag_ok, st.piso_ok, st.crc_err, st.bytes_descartados);
  return 0;
}
