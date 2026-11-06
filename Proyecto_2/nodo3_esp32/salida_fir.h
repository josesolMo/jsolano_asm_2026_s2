// Salida temporal (metodo del nodo 2) dentro del nodo 3: filtro FIR de fase lineal con los
// perfiles de nodo2_temporal/coeficientes.h. Permite escuchar en el mismo parlante la salida
// espectral ('mode filter') o la temporal ('mode fir') y comparar con la entrada ('mode raw').
//
// Alineacion: un FIR simetrico de T coeficientes retarda D = (T-1)/2 muestras. Se agrega un
// retardo de HOP - D para que la salida quede retardada exactamente HOP, como 'raw' y 'filter';
// asi las metricas con referencia (que asumen y[t] ~ x[t-HOP]) siguen siendo validas.
#pragma once
#include <math.h>
#include <string.h>
#include "config.h"
#include "fft_processing.h"
#include "spectral_filter.h"
#include "filtro_fir.h"
#include "coeficientes.h"

namespace n3 {

enum class PerfilFir : uint8_t { Tono = 0, Multitono = 1, Audio = 2, Transitoria = 3 };

inline const char* nombre_perfil(PerfilFir p) {
  switch (p) {
    case PerfilFir::Tono: return "tono";
    case PerfilFir::Multitono: return "multitono";
    case PerfilFir::Audio: return "audio";
    default: return "transitoria";
  }
}

class SalidaFir {
 public:
  void set_perfil(PerfilFir p) { perfil_ = p; reiniciar(); }
  PerfilFir perfil() const { return perfil_; }
  int retardo() const {
    switch (perfil_) {
      case PerfilFir::Tono: return int(temporal::TONO.size() - 1) / 2;
      case PerfilFir::Multitono: return int(temporal::MULTITONO.size() - 1) / 2;
      case PerfilFir::Audio: return int(temporal::AUDIO.size() - 1) / 2;
      default: return int(temporal::TRANSITORIA.size() - 1) / 2;
    }
  }
  void reiniciar() {
    tono_.reiniciar(); multi_.reiniciar(); audio_.reiniciar(); trans_.reiniciar();
    memset(cola_, 0, sizeof cola_);
  }
  // Filtra HOP muestras nuevas y entrega HOP muestras retardadas exactamente HOP.
  void procesar(const float* x, float* y) {
    memmove(cola_, cola_ + HOP, HOP * sizeof(float));
    float* nueva = cola_ + HOP;
    switch (perfil_) {
      case PerfilFir::Tono: tono_.procesar(x, nueva, HOP); break;
      case PerfilFir::Multitono: multi_.procesar(x, nueva, HOP); break;
      case PerfilFir::Audio: audio_.procesar(x, nueva, HOP); break;
      default: trans_.procesar(x, nueva, HOP); break;
    }
    // fir[t] ~ x[t - D]  =>  y[t] = x[t - HOP] = fir[t - HOP + D]  (indice i + D en cola_)
    memcpy(y, cola_ + retardo(), HOP * sizeof(float));
  }

 private:
  temporal::FiltroFIR<temporal::TONO.size()> tono_{temporal::TONO};
  temporal::FiltroFIR<temporal::MULTITONO.size()> multi_{temporal::MULTITONO};
  temporal::FiltroFIR<temporal::AUDIO.size()> audio_{temporal::AUDIO};
  temporal::FiltroFIR<temporal::TRANSITORIA.size()> trans_{temporal::TRANSITORIA};
  PerfilFir perfil_ = PerfilFir::Multitono;
  float cola_[2 * HOP] = {};
};

// Metricas sin referencia para una salida que no pasa por el filtro espectral (FIR):
// mismas magnitudes que SpectralNode::advance, pero midiendo la salida con su propia FFT.
// Entrada y salida deben estar alineadas (ambas retardadas HOP).
class StatsSenales {
 public:
  bool begin() {
    if (!fft_.begin()) return false;
    make_sqrt_hann(w_, N);
    double s = 0;
    for (int i = 0; i < N; ++i) s += double(w_[i]) * w_[i];
    norma_ = float(s);
    reset();
    return true;
  }
  void reset() { memset(xin_, 0, sizeof xin_); memset(xout_, 0, sizeof xout_); }
  void on_hop(const float* x, const float* y, const Diag* diag, const Params& p, HopStats* st) {
    memmove(xin_, xin_ + HOP, HOP * sizeof(float)); memcpy(xin_ + HOP, x, HOP * sizeof(float));
    memmove(xout_, xout_ + HOP, HOP * sizeof(float)); memcpy(xout_ + HOP, y, HOP * sizeof(float));
    potencia(xin_, pin_);
    potencia(xout_, pout_);
    const int u0 = bin(p.banda_lo), u1 = bin(p.banda_hi), r0 = bin(p.rint_lo), r1 = bin(p.rint_hi);
    HopStats h;
    for (int k = 0; k < NB; ++k) {
      const float P = pin_[k], Po = pout_[k];
      const float g2 = P > 1e-20f ? Po / P : 1.0f;
      const float piso = (diag && diag->piso_valido) ? diag->piso[k] : 0.0f;
      h.t_in += P; h.t_out += Po;
      h.piso_in += piso; h.piso_out += piso * g2;
      if (g2 < 0.25f) ++h.bins_atenuados;                      // |G| < 0.5
      if (k >= u0 && k <= u1) {
        const float s = P - piso > 0.0f ? P - piso : 0.0f;
        h.p_util_in += P; h.p_util_out += Po;
        h.s_in += s; h.s_out += s * g2;
      }
      if (k >= r0 && k <= r1) { h.p_rng_in += P; h.p_rng_out += Po; }
    }
    *st = h;
  }

 private:
  static int bin(float hz) {
    const int b = int(lrintf(hz / BIN_HZ));
    return b < 0 ? 0 : (b > NB - 1 ? NB - 1 : b);
  }
  void potencia(const float* x, float* P) {
    for (int i = 0; i < N; ++i) { buf_[2 * i] = x[i] * w_[i]; buf_[2 * i + 1] = 0.0f; }
    fft_.forward(buf_);
    const float inv = 1.0f / norma_;
    for (int k = 0; k < NB; ++k) P[k] = (buf_[2 * k] * buf_[2 * k] + buf_[2 * k + 1] * buf_[2 * k + 1]) * inv;
  }
  Fft fft_;
  alignas(16) float buf_[2 * N];
  float w_[N], xin_[N], xout_[N], pin_[NB], pout_[NB];
  float norma_ = 1.0f;
};

}  // namespace n3
