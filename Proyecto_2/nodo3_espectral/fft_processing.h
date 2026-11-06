// FFT/IFFT complejas de N = 1024 puntos en float32.
// En la ESP32 usa esp-dsp (incluida en el nucleo Arduino-ESP32 3.3.x, rutina
// ensamblada ae32). En el PC (pruebas, -DNODO3_HOST) usa un radix-2 propio.
#pragma once
#include "config.h"

namespace n3 {

class Fft {
 public:
  // Prepara la tabla de twiddles. Devuelve false si el backend falla.
  bool begin();
  // data: N puntos complejos intercalados [re0, im0, re1, im1, ...], alineado a 16 bytes.
  // Calculo en sitio, salida en orden natural.
  void forward(float* data);
  // Inversa en sitio, ya dividida entre N.
  void inverse(float* data);
  const char* backend() const;
};

// Ventana raiz de Hann periodica: w[n] = sqrt(0.5 - 0.5 cos(2 pi n / N)).
void make_sqrt_hann(float* w, int n);
// Ventana de Hann periodica (la que usa el diagnostico del Nodo 1).
void make_hann(float* w, int n);

}  // namespace n3
