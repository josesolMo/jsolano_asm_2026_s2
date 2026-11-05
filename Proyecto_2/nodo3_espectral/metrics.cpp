#include "metrics.h"
#include <math.h>
#include <string.h>

namespace n3 {

void Metrics::reset() {
  idx_ = ref_n_ = hops_ = 0;
  ss_ = e_in_ = e_out_ = sy_ = 0;
  rng_in_ = rng_out_ = util_in_ = util_out_ = 0;
  s_in_ = t_in_ = s_out_ = t_out_ = piso_in_ = piso_out_ = atenuados_ = 0;
  t_blocks = 0; t_min_us = 0xFFFFFFFF; t_max_us = 0; t_sum_us = 0; t_excedidos = 0;
}

void Metrics::on_hop(const HopStats& h) {
  rng_in_ += h.p_rng_in; rng_out_ += h.p_rng_out;
  util_in_ += h.p_util_in; util_out_ += h.p_util_out;
  s_in_ += h.s_in; t_in_ += h.t_in; s_out_ += h.s_out; t_out_ += h.t_out;
  piso_in_ += h.piso_in; piso_out_ += h.piso_out;
  atenuados_ += h.bins_atenuados;
  ++hops_;
}

void Metrics::on_samples(const float* s, const float* x, const float* y, int n) {
  for (int i = 0; i < n; ++i, ++idx_) {
    if (idx_ < uint32_t(N) || idx_ >= uint32_t(TEST_MUESTRAS - N)) continue;   // transitorios, como en Python
    const double sv = s[i], ev_in = sv - x[i], ev_out = sv - y[i];
    ss_ += sv * sv; e_in_ += ev_in * ev_in; e_out_ += ev_out * ev_out; sy_ += sv * y[i];
    ++ref_n_;
  }
}

Reporte Metrics::reporte() const {
  Reporte r;
  r.hops = hops_;
  if (ref_n_ > 0 && ss_ > 0 && e_in_ > 0 && e_out_ > 0) {
    r.ref_muestras = ref_n_;
    r.mse = e_out_ / ref_n_;
    r.snr_in = 10 * log10(ss_ / e_in_);
    r.snr_out = 10 * log10(ss_ / e_out_);
    r.dsnr = r.snr_out - r.snr_in;
    const double g = sy_ / ss_;                 // proyeccion de la salida sobre la referencia
    r.e_util = 100.0 * g * g;
  }
  if (hops_ > 0) {
    if (rng_in_ > 0 && rng_out_ > 0) r.atenuacion_rng_db = 10 * log10(rng_in_ / rng_out_);
    if (util_in_ > 0) r.util_conservada_pct = 100.0 * util_out_ / util_in_;
    if (s_in_ > 0 && t_in_ > s_in_) r.snr_est_in_db = 10 * log10(s_in_ / (t_in_ - s_in_));
    if (s_out_ > 0 && t_out_ > s_out_) r.snr_est_out_db = 10 * log10(s_out_ / (t_out_ - s_out_));
    if (piso_in_ > 0 && piso_out_ > 0) r.reduccion_piso_db = 10 * log10(piso_in_ / piso_out_);
    r.bins_atenuados_medio = atenuados_ / hops_;
  }
  return r;
}

void Metrics::on_block_time(uint32_t us) {
  ++t_blocks; t_sum_us += us;
  if (us < t_min_us) t_min_us = us;
  if (us > t_max_us) t_max_us = us;
  if (us > 1000u * N / (FS / 1000)) ++t_excedidos;     // > 64 000 us
}

}  // namespace n3
