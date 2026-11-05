// Filtro espectral adaptativo (port de full_system/sistema_ruido/spectral.py) y
// estimador local de diagnostico (port de acquisition.py, Nodo 1).
#pragma once
#include "config.h"
#include "fft_processing.h"

namespace n3 {

// Diagnostico que consume el filtro (mismo contenido que Diagnostico en Python).
struct Diag {
  float piso[NB];                 // PSD de ruido por bin (misma normalizacion que el periodograma)
  uint16_t interf[MAX_INTERF];    // bins interferentes
  uint8_t n_interf = 0;
  bool piso_valido = false;       // false hasta recibir/estimar un piso
};

// Magnitudes espectrales de un avance (para metricas sin referencia).
struct HopStats {
  float p_rng_in = 0, p_rng_out = 0;      // potencia en el rango de interferencia
  float p_util_in = 0, p_util_out = 0;    // potencia en la banda util
  float s_in = 0, t_in = 0;               // senal estimada y total (entrada)
  float s_out = 0, t_out = 0;             // idem a la salida del filtro
  float piso_in = 0, piso_out = 0;        // potencia total del piso estimado, antes/despues
  int bins_atenuados = 0;                 // bins con G < 0.5
};

class SpectralNode {
 public:
  bool begin();
  void reset();
  // Consume HOP muestras (float, escala +-1) y entrega HOP muestras con un retardo de HOP.
  // diag == nullptr o modo Bypass => ganancia espectral unitaria.
  void advance(const float* nuevo, const Diag* diag, const Params& p, float* salida,
               HopStats* st = nullptr);
  const char* backend() const { return fft_.backend(); }
  float last_gain(int k) const { return gain_[k]; }
 private:
  Fft fft_;
  alignas(16) float buf_[2 * N];
  float w_[N];
  float prev_[HOP];
  float ola_[HOP];
  float gain_[NB];
  float norma_ = 1.0f;
};

// Estimador local: quita DC, Hann, FFT, EMA del espectro, piso por mediana, picos e interferentes.
class DiagLocal {
 public:
  bool begin();
  void reset() { primer_ = true; }
  void process(const float* bloque, const Params& p, Diag& out);
  const float* psd() const { return psd_; }
 private:
  Fft fft_;
  alignas(16) float buf_[2 * N];
  float w_[N];
  float psd_[NB];
  float norma_ = 1.0f;
  bool primer_ = true;
};

// Utilidades para el diagnostico recibido del Nodo 1.
void diag_set_interf_hz(Diag& d, const float* f_hz, const uint8_t* flags, int n);
void diag_set_piso_knots(Diag& d, const int16_t* centi_db, int nk, int decim);

}  // namespace n3
