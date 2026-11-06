// Pasa altos de entrada (comando 'hpf <Hz>', 0 = apagado; defecto 100 Hz).
// Butterworth de 2o orden (biquad RBJ, Q = 1/sqrt(2)), forma directa II transpuesta en float.
// Quita el DC (adcmid desajustado) y el ruido grave del microfono y la alimentacion (0-200 Hz, visto
// en capturas en silencio). Ninguna prueba tiene senal util bajo 150 Hz (P3 empieza ahi): en 150 Hz
// atenua ~1 dB y en 300 Hz < 0.1 dB. Se aplica a la entrada, antes de las dos rutas, asi el FIR del
// Nodo 2 y el filtro espectral reciben la misma senal.
#pragma once
#include <math.h>
#include "config.h"

namespace n3 {

class PasaAltos {
 public:
  void configurar(float fc_hz) {
    fc_ = fc_hz;
    reset();
    if (fc_hz <= 0.0f) return;
    const double w0 = 2.0 * M_PI * fc_hz / FS, c = cos(w0), alfa = sin(w0) / (2.0 * M_SQRT1_2);
    const double a0 = 1.0 + alfa;
    b0_ = float((1.0 + c) / 2.0 / a0);
    b1_ = float(-(1.0 + c) / a0);
    b2_ = b0_;
    a1_ = float(-2.0 * c / a0);
    a2_ = float((1.0 - alfa) / a0);
  }
  void procesar(float* x, int n) {
    if (fc_ <= 0.0f) return;
    for (int i = 0; i < n; ++i) {
      const float in = x[i];
      const float out = b0_ * in + z1_;
      z1_ = b1_ * in - a1_ * out + z2_;
      z2_ = b2_ * in - a2_ * out;
      x[i] = out;
    }
  }
  void reset() { z1_ = z2_ = 0.0f; }
  float fc() const { return fc_; }

 private:
  float fc_ = 0.0f, b0_ = 1, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0, z1_ = 0, z2_ = 0;
};

}  // namespace n3
