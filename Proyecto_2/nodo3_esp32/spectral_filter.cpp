#include "spectral_filter.h"
#include <algorithm>
#include <math.h>
#include <string.h>

namespace n3 {

namespace {
inline int bin_de_hz(float hz) {
  int b = int(lrintf(hz / BIN_HZ));
  return b < 0 ? 0 : (b > NB - 1 ? NB - 1 : b);
}
}  // namespace

// ---------------------------------------------------------------- SpectralNode
bool SpectralNode::begin() {
  if (!fft_.begin()) return false;
  make_sqrt_hann(w_, N);
  double s = 0;
  for (int i = 0; i < N; ++i) s += double(w_[i]) * w_[i];
  norma_ = float(s);
  reset();
  return true;
}

void SpectralNode::reset() {
  memset(prev_, 0, sizeof prev_);
  memset(ola_, 0, sizeof ola_);
  for (int k = 0; k < NB; ++k) gain_[k] = 1.0f;
}

void SpectralNode::advance(const float* nuevo, const Diag* diag, const Params& p, float* salida,
                           HopStats* st) {
  // 1-2. ultimas N muestras = [anterior | nuevo], ventana raiz de Hann, FFT compleja
  for (int i = 0; i < HOP; ++i) {
    buf_[2 * i] = prev_[i] * w_[i];
    buf_[2 * i + 1] = 0.0f;
    buf_[2 * (HOP + i)] = nuevo[i] * w_[HOP + i];
    buf_[2 * (HOP + i) + 1] = 0.0f;
  }
  memcpy(prev_, nuevo, sizeof prev_);
  fft_.forward(buf_);

  // 3. potencia y ganancia real por bin
  // la ruta espectral filtra siempre (las dos rutas se miden a la vez); 'bypass' deja G = 1
  const bool filtrar = diag && diag->piso_valido && p.modo != Modo::Bypass;
  const float inv_norma = 1.0f / norma_;
  const float beta2 = p.beta * p.beta;
  for (int k = 0; k < NB; ++k) {
    float g = 1.0f;
    if (filtrar) {
      const float P = (buf_[2 * k] * buf_[2 * k] + buf_[2 * k + 1] * buf_[2 * k + 1]) * inv_norma;
      float piso = diag->piso[k];
      if (!(piso >= 0.0f) || !isfinite(piso)) piso = 0.0f;    // NaN/inf/negativo => sin sustraccion
      float r = 1.0f - p.alfa * piso / (P + 1e-20f);
      if (!(r >= beta2)) r = beta2;                            // tambien captura NaN
      if (r > 1.0f) r = 1.0f;
      g = sqrtf(r);
    }
    gain_[k] = g;
  }
  // 4. rechazo de interferentes identificados
  if (filtrar) {
    for (int i = 0; i < diag->n_interf; ++i) {
      const int b = diag->interf[i];
      const int lo = std::max(0, b - p.notch_bins), hi = std::min(NB - 1, b + p.notch_bins);
      for (int k = lo; k <= hi; ++k) gain_[k] = p.g_interf;
    }
  }

  // Estadisticas espectrales (antes de aplicar la ganancia).
  if (st) {
    const int u0 = bin_de_hz(p.banda_lo), u1 = bin_de_hz(p.banda_hi);
    const int r0 = bin_de_hz(p.rint_lo), r1 = bin_de_hz(p.rint_hi);
    HopStats h;
    for (int k = 0; k < NB; ++k) {
      const float P = (buf_[2 * k] * buf_[2 * k] + buf_[2 * k + 1] * buf_[2 * k + 1]) * inv_norma;
      const float g2 = gain_[k] * gain_[k], Po = P * g2;
      const float piso = (diag && diag->piso_valido) ? diag->piso[k] : 0.0f;
      h.t_in += P; h.t_out += Po;
      h.piso_in += piso; h.piso_out += piso * g2;
      if (gain_[k] < 0.5f) ++h.bins_atenuados;
      if (k >= u0 && k <= u1) {
        const float s = std::max(P - piso, 0.0f);
        h.p_util_in += P; h.p_util_out += Po;
        h.s_in += s; h.s_out += s * g2;
      }
      if (k >= r0 && k <= r1) { h.p_rng_in += P; h.p_rng_out += Po; }
    }
    *st = h;
  }

  // 5. Y = X G con simetria conjugada explicita; IFFT por conjugacion
  for (int k = 0; k < NB; ++k) {
    buf_[2 * k] *= gain_[k];
    buf_[2 * k + 1] *= gain_[k];
  }
  buf_[1] = 0.0f;                    // DC real
  buf_[2 * (N / 2) + 1] = 0.0f;      // Nyquist real
  for (int k = 1; k < N / 2; ++k) {
    buf_[2 * (N - k)] = buf_[2 * k];
    buf_[2 * (N - k) + 1] = -buf_[2 * k + 1];
  }
  fft_.inverse(buf_);

  // 6-7. ventana de sintesis y overlap-add
  for (int i = 0; i < HOP; ++i) {
    salida[i] = ola_[i] + buf_[2 * i] * w_[i];
    ola_[i] = buf_[2 * (HOP + i)] * w_[HOP + i];
  }
}

// ------------------------------------------------------------------- DiagLocal
bool DiagLocal::begin() {
  if (!fft_.begin()) return false;
  make_hann(w_, N);
  double s = 0;
  for (int i = 0; i < N; ++i) s += double(w_[i]) * w_[i];
  norma_ = float(s);
  reset();
  return true;
}

void DiagLocal::process(const float* bloque, const Params& p, Diag& out) {
  double media = 0;
  for (int i = 0; i < N; ++i) media += bloque[i];
  media /= N;
  for (int i = 0; i < N; ++i) {
    buf_[2 * i] = float(bloque[i] - media) * w_[i];
    buf_[2 * i + 1] = 0.0f;
  }
  fft_.forward(buf_);

  const float inv = 1.0f / norma_;
  for (int k = 0; k < NB; ++k) {
    const float P = (buf_[2 * k] * buf_[2 * k] + buf_[2 * k + 1] * buf_[2 * k + 1]) * inv;
    psd_[k] = primer_ ? P : p.lambda_ema * psd_[k] + (1.0f - p.lambda_ema) * P;
  }
  primer_ = false;

  // piso = mediana deslizante de K_MEDIANA bins con reflexion (scipy mode="reflect")
  constexpr int H = K_MEDIANA / 2;
  float ventana[K_MEDIANA];
  for (int i = 0; i < NB; ++i) {
    for (int j = 0; j < K_MEDIANA; ++j) {
      int idx = i + j - H;
      if (idx < 0) idx = -idx - 1;
      if (idx >= NB) idx = 2 * NB - 1 - idx;
      ventana[j] = psd_[idx];
    }
    std::nth_element(ventana, ventana + H, ventana + K_MEDIANA);
    out.piso[i] = ventana[H];
  }
  out.piso_valido = true;

  // picos: maximos locales 15 dB sobre el piso y a menos de 40 dB del mayor
  const float umbral = powf(10.0f, UMBRAL_PICO_DB / 10.0f);
  uint16_t pic[NB];
  int np = 0;
  float pmax = 0;
  for (int i = 1; i < NB - 1; ++i) {
    if (psd_[i] > psd_[i - 1] && psd_[i] >= psd_[i + 1] && psd_[i] > out.piso[i] * umbral) {
      pic[np++] = uint16_t(i);
      pmax = std::max(pmax, psd_[i]);
    }
  }
  const float corte = pmax * powf(10.0f, -RANGO_PICO_DB / 10.0f);
  int m = 0;
  for (int i = 0; i < np; ++i)
    if (psd_[pic[i]] >= corte) pic[m++] = pic[i];
  np = m;

  // interferente = fuera de la banda util (con guarda) o zumbido de red (60 Hz y armonicos)
  bool fuera[NB], red[NB];
  bool hay_k1 = false;
  int n_red = 0;
  for (int i = 0; i < np; ++i) {
    const float f = pic[i] * BIN_HZ;
    fuera[i] = (f < p.banda_lo - GUARDA_BINS * BIN_HZ) || (f > p.banda_hi + GUARDA_BINS * BIN_HZ);
    const float k = nearbyintf(f / F_RED);
    red[i] = (k >= 1 && k <= 8 && fabsf(f - k * F_RED) < 0.75f * BIN_HZ);
    if (red[i]) { ++n_red; if (k == 1) hay_k1 = true; }
  }
  const bool zumbido = (n_red >= 2) && hay_k1;
  out.n_interf = 0;
  for (int i = 0; i < np && out.n_interf < MAX_INTERF; ++i)
    if (fuera[i] || (red[i] && zumbido)) out.interf[out.n_interf++] = pic[i];
}

// ------------------------------------------------- diagnostico recibido (Nodo 1)
void diag_set_interf_hz(Diag& d, const float* f_hz, const uint8_t* flags, int n) {
  d.n_interf = 0;
  for (int i = 0; i < n && d.n_interf < MAX_INTERF; ++i)
    if (flags[i] & 0x01) d.interf[d.n_interf++] = uint16_t(bin_de_hz(f_hz[i]));   // FLAG_INTERFERENTE
}

void diag_set_piso_knots(Diag& d, const int16_t* cdb, int nk, int decim) {
  if (nk < 2 || decim < 1) return;
  for (int k = 0; k < NB; ++k) {
    int j = k / decim;
    float fr = float(k - j * decim) / decim;
    if (j >= nk - 1) { j = nk - 2; fr = 1.0f; }
    const float db = (cdb[j] + fr * (cdb[j + 1] - cdb[j])) * 0.01f;
    d.piso[k] = powf(10.0f, db * 0.1f);
  }
  d.piso_valido = true;
}

}  // namespace n3
