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
#include "../metrics.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
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
  } else if (!strcmp(cmd, "knots")) {
    // knots in.i16 diag.bin out.f32 : diagnostico por bloque como lo recibiria del Nodo 1:
    //   u8 decim, u8 nk, i16 centi_db[nk], u8 n, n x (f32 f_hz, u8 flags)
    FILE* fd = fopen(argv[3], "rb"); if (!fd) { perror("diag"); return 2; }
    for (size_t b = 0; b < nb; ++b) {
      static Diag d; uint8_t decim, nk, n; int16_t cdb[PISO_MAX_KNOTS]; float f[MAX_FREQ]; uint8_t fl[MAX_FREQ];
      if (fread(&decim, 1, 1, fd) != 1 || fread(&nk, 1, 1, fd) != 1 || fread(cdb, 2, nk, fd) != nk || fread(&n, 1, 1, fd) != 1) return 2;
      for (int i = 0; i < n; ++i) if (fread(&f[i], 4, 1, fd) != 1 || fread(&fl[i], 1, 1, fd) != 1) return 2;
      diag_set_piso_knots(d, cdb, nk, decim); diag_set_interf_hz(d, f, fl, n);
      float x[N]; for (int i = 0; i < N; ++i) x[i] = in[b * N + i] / 32768.0f;
      for (int h = 0; h < 2; ++h) { float y[HOP]; nodo.advance(x + h * HOP, &d, p, y); out.insert(out.end(), y, y + HOP); }
    }
    escribir_f32(argv[4], out);
  } else if (!strcmp(cmd, "metr")) {
    // metr in.i16 ref.f32 banda_lo banda_hi rint_lo rint_hi : imprime el Reporte de Metrics
    DiagLocal dl; dl.begin(); Metrics m; m.reset();
    p.banda_lo = atof(argv[4]); p.banda_hi = atof(argv[5]); p.rint_lo = atof(argv[6]); p.rint_hi = atof(argv[7]);
    FILE* fr = fopen(argv[3], "rb"); std::vector<float> ref(in.size());
    if (fread(ref.data(), 4, ref.size(), fr) != ref.size()) return 2;
    fclose(fr);
    for (size_t b = 0; b < nb; ++b) {
      static Diag d; float x[N]; for (int i = 0; i < N; ++i) x[i] = in[b * N + i] / 32768.0f;
      dl.process(x, p, d);
      for (int h = 0; h < 2; ++h) {
        float y[HOP]; HopStats hs; nodo.advance(x + h * HOP, &d, p, y, &hs); m.on_hop(hs);
        out.insert(out.end(), y, y + HOP);
      }
    }
    std::vector<float> xs(in.size()); for (size_t i = 0; i < in.size(); ++i) xs[i] = in[i] / 32768.0f;
    for (size_t t = 0; t + HOP < out.size() && t < in.size(); t += HOP)       // salida alineada: y[t] = out[t+HOP]
      m.on_samples(ref.data() + t, xs.data() + t, out.data() + t + HOP, int(std::min<size_t>(HOP, in.size() - t)));
    Reporte r = m.reporte();
    printf("mse=%.9e snr_in=%.6f snr_out=%.6f dsnr=%.6f e_util=%.6f aten=%.6f util=%.6f snr_est_in=%.6f snr_est_out=%.6f red_piso=%.6f muestras=%u\n",
           r.mse, r.snr_in, r.snr_out, r.dsnr, r.e_util, r.atenuacion_rng_db, r.util_conservada_pct,
           r.snr_est_in_db, r.snr_est_out_db, r.reduccion_piso_db, r.ref_muestras);
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
