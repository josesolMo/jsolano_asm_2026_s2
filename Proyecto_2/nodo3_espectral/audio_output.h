// Salida DAC continua temporizada por hardware (I2S/DMA, reloj APLL), desacoplada del
// procesamiento mediante un anillo SPSC: el calculo FFT puede variar sin alterar la fs de salida.
#pragma once
#include <atomic>
#include <string.h>
#include "config.h"

namespace n3 {

// Anillo de una sola productora y un solo consumidor (tareas en nucleos distintos).
class SampleRing {
 public:
  size_t size() const { return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire); }
  size_t libre() const { return RING_SAMPLES - size(); }
  // Productora: devuelve cuantas muestras entraron (menos que n => desbordamiento).
  size_t push(const uint8_t* d, size_t n) {
    const size_t k = n < libre() ? n : libre();
    size_t h = head_.load(std::memory_order_relaxed);
    for (size_t i = 0; i < k; ++i) buf_[(h + i) & (RING_SAMPLES - 1)] = d[i];
    head_.store(h + k, std::memory_order_release);
    return k;
  }
  // Consumidora: devuelve cuantas muestras salieron.
  size_t pop(uint8_t* d, size_t n) {
    const size_t k = n < size() ? n : size();
    size_t t = tail_.load(std::memory_order_relaxed);
    for (size_t i = 0; i < k; ++i) d[i] = buf_[(t + i) & (RING_SAMPLES - 1)];
    tail_.store(t + k, std::memory_order_release);
    return k;
  }
  void clear() { tail_.store(head_.load()); }
 private:
  uint8_t buf_[RING_SAMPLES];
  std::atomic<size_t> head_{0}, tail_{0};
};

// float (+-1.0 = escala completa) -> codigo de 8 bits con saturacion y centro 128.
inline uint8_t a_codigo_dac(float y, float ganancia, uint32_t& saturadas) {
  float v = (y == y) ? y * ganancia + float(DAC_CENTRO) : float(DAC_CENTRO);   // NaN => centro
  if (v < 0.0f) { ++saturadas; return 0; }
  if (v > 255.0f) { ++saturadas; return 255; }
  return uint8_t(v + 0.5f);
}

struct AudioStats {
  uint32_t escritas = 0;        // muestras entregadas al DMA
  uint32_t desbordes = 0;       // muestras descartadas por anillo lleno
  uint32_t vacios = 0;          // veces que el DAC se quedo sin datos (relleno con centro)
  uint32_t muestras_vacias = 0;
  uint32_t saturadas = 0;
  uint32_t errores_dac = 0;
  bool activo = false;
};

#ifndef NODO3_HOST
// Control del DAC. begin() deja la salida en el centro; el DMA arranca solo cuando el anillo
// acumula RING_PREFILL muestras y se detiene con stop().
class AudioOut {
 public:
  bool begin();                              // crea la tarea del DAC y fija el reposo (128)
  size_t escribir(const float* y, size_t n); // convierte, satura y encola; devuelve las encoladas
  size_t escribir_codigos(const uint8_t* c, size_t n);
  void stop();
  void set_ganancia(float g) { ganancia_ = g; }
  float ganancia() const { return ganancia_; }
  AudioStats stats;
  SampleRing ring;
 private:
  friend void tarea_dac(void*);
  bool iniciar_dma();
  void detener_dma();
  void* continuo_ = nullptr;                 // dac_continuous_handle_t
  void* reposo_ = nullptr;                   // dac_oneshot_handle_t
  volatile bool quiere_ = true;
  float ganancia_ = DAC_GANANCIA;
};
#endif

}  // namespace n3
