// Harness de pruebas en el PC: compila el mismo DSP del firmware (backend radix-2).
// Subcomandos (binarios little-endian):
//   fft   in.f32           out.f32          FFT y IFFT de un bloque real de N muestras
//   ext   in.i16 diag.bin  out.f32 [modo]   filtro con diagnostico externo (piso+interf de Python)
//   local in.i16 out.f32 lo hi diag_out.bin filtro con estimador local; vuelca piso e interf
// diag.bin por bloque de N: float32 piso[513], uint16 n, uint16 interf[n]
#ifndef NODO3_HOST
#define NODO3_HOST 1
#endif
#include "../spectral_filter.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string.h>
using namespace n3;

static std::vector<int16_t> leer_i16(const char* f) {
  FILE* fp = fopen(f, "rb"); if (!fp) { perror(f); exit(2); }
  fseek(fp, 0, SEEK_END); long n = ftell(fp) / 2; fseek(fp, 0, SEEK_SET);
  std::vector<int16_t> v(n); if (fread(v.data(), 2, n, fp) != size_t(n)) exit(2); fclose(fp); return v;
}
static void escribir_f32(const char* f, const std::vector<float>& v) {
  FILE* fp = fopen(f, "wb"); fwrite(v.data(), 4, v.size(), fp); fclose(fp);
}

int main(int argc, char** argv) {
  if (argc < 2) return 1;
  const char* cmd = argv[1];
  if (!strcmp(cmd, "fft")) {
    FILE* fp = fopen(argv[2], "rb"); std::vector<float> x(N);
    if (fread(x.data(), 4, N, fp) != size_t(N)) return 2;
    fclose(fp);
    Fft fft; fft.begin();
    alignas(16) static float d[2 * N];
    for (int i = 0; i < N; ++i) { d[2 * i] = x[i]; d[2 * i + 1] = 0; }
    fft.forward(d);
    std::vector<float> out(d, d + 2 * N);
    fft.inverse(d);                       // IFFT(FFT(x)) debe devolver x
    out.insert(out.end(), d, d + 2 * N);
    escribir_f32(argv[3], out);
    return 0;
  }
  Params p;
  SpectralNode nodo; nodo.begin();
  auto in = leer_i16(argv[2]);
  const size_t nb = in.size() / N;
  std::vector<float> out;
  if (!strcmp(cmd, "ext")) {
    FILE* fd = fopen(argv[3], "rb"); if (!fd) { perror("diag"); return 2; }
    if (argc > 5) p.modo = (Modo)atoi(argv[5]);
    for (size_t b = 0; b < nb; ++b) {
      static Diag d; uint16_t n;
      if (fread(d.piso, 4, NB, fd) != size_t(NB) || fread(&n, 2, 1, fd) != 1) return 2;
      d.n_interf = (uint8_t)n; if (n && fread(d.interf, 2, n, fd) != n) return 2;
      d.piso_valido = true;
      float x[N]; for (int i = 0; i < N; ++i) x[i] = in[b * N + i] / 32768.0f;
      for (int h = 0; h < 2; ++h) { float y[HOP]; nodo.advance(x + h * HOP, &d, p, y); out.insert(out.end(), y, y + HOP); }
    }
    escribir_f32(argv[4], out);
  } else if (!strcmp(cmd, "local")) {
    DiagLocal dl; dl.begin();
    p.banda_lo = atof(argv[4]); p.banda_hi = atof(argv[5]);
    FILE* fd = fopen(argv[6], "wb");
    for (size_t b = 0; b < nb; ++b) {
      static Diag d; float x[N]; for (int i = 0; i < N; ++i) x[i] = in[b * N + i] / 32768.0f;
      dl.process(x, p, d);
      uint16_t n = d.n_interf;
      fwrite(d.piso, 4, NB, fd); fwrite(&n, 2, 1, fd); fwrite(d.interf, 2, n, fd);
      for (int h = 0; h < 2; ++h) { float y[HOP]; nodo.advance(x + h * HOP, &d, p, y); out.insert(out.end(), y, y + HOP); }
    }
    fclose(fd);
    escribir_f32(argv[3], out);
  }
  return 0;
}
