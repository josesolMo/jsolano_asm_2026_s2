#include "fft_processing.h"
#include <math.h>

#ifndef NODO3_HOST
#include "dsps_fft2r.h"
#endif

namespace n3 {

#ifndef NODO3_HOST
// ---- Backend ESP32: esp-dsp --------------------------------------------------
namespace {
float g_tabla[N] __attribute__((aligned(16)));   // N/2 twiddles complejos = N floats
bool g_listo = false;
}

bool Fft::begin() {
  if (g_listo) return true;
  g_listo = (dsps_fft2r_init_fc32(g_tabla, N) == ESP_OK);
  return g_listo;
}
void Fft::forward(float* data) {
  dsps_fft2r_fc32(data, N);
  dsps_bit_rev_fc32(data, N);
}
const char* Fft::backend() const { return "esp-dsp fc32 (ae32)"; }

#else
// ---- Backend PC: radix-2 iterativo en float32 --------------------------------
namespace {
float g_cos[N / 2], g_sin[N / 2];
uint16_t g_rev[N];
bool g_listo = false;
}

bool Fft::begin() {
  if (g_listo) return true;
  for (int k = 0; k < N / 2; ++k) {
    const double a = -2.0 * M_PI * k / N;
    g_cos[k] = float(cos(a));
    g_sin[k] = float(sin(a));
  }
  for (int i = 0; i < N; ++i) {
    int r = 0;
    for (int b = 0; b < 10; ++b) r |= ((i >> b) & 1) << (9 - b);
    g_rev[i] = uint16_t(r);
  }
  g_listo = true;
  return true;
}
void Fft::forward(float* d) {
  for (int i = 0; i < N; ++i) {
    const int j = g_rev[i];
    if (j > i) {
      float t = d[2 * i]; d[2 * i] = d[2 * j]; d[2 * j] = t;
      t = d[2 * i + 1]; d[2 * i + 1] = d[2 * j + 1]; d[2 * j + 1] = t;
    }
  }
  for (int len = 2; len <= N; len <<= 1) {
    const int half = len / 2, paso = N / len;
    for (int i = 0; i < N; i += len) {
      for (int k = 0; k < half; ++k) {
        const float wr = g_cos[k * paso], wi = g_sin[k * paso];
        float* a = d + 2 * (i + k);
        float* b = d + 2 * (i + k + half);
        const float tr = b[0] * wr - b[1] * wi;
        const float ti = b[0] * wi + b[1] * wr;
        b[0] = a[0] - tr; b[1] = a[1] - ti;
        a[0] += tr; a[1] += ti;
      }
    }
  }
}
const char* Fft::backend() const { return "radix-2 float32 (PC)"; }
#endif

// IFFT(X) = conj(FFT(conj(X))) / N: reutiliza la rutina directa.
void Fft::inverse(float* data) {
  for (int i = 0; i < N; ++i) data[2 * i + 1] = -data[2 * i + 1];
  forward(data);
  const float s = 1.0f / N;
  for (int i = 0; i < N; ++i) {
    data[2 * i] *= s;
    data[2 * i + 1] *= -s;
  }
}

void make_hann(float* w, int n) {
  for (int i = 0; i < n; ++i) w[i] = float(0.5 - 0.5 * cos(2.0 * M_PI * i / n));
}
void make_sqrt_hann(float* w, int n) {
  for (int i = 0; i < n; ++i) w[i] = float(sqrt(0.5 - 0.5 * cos(2.0 * M_PI * i / n)));
}

}  // namespace n3
