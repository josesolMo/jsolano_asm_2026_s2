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

// Compensacion de reloj entre nodos. El ADC del Nodo 1 y el DAC de este nodo usan cristales
// distintos (medido: ~0.36 % de diferencia); sin correccion el anillo se llena (o se vacia) y se
// pierden muestras (chasquidos). Se remuestrea con interpolacion lineal a razon r = 1 + e, con e
// proporcional a cuanto se aparta el anillo de su nivel objetivo (control P, suavizado).
// e ~ 0.36 % equivale a 0.06 semitonos: inaudible.
class AjusteReloj {
 public:
  static constexpr float OBJETIVO = 768.0f;    // cuadros en el anillo (48 ms)
  static constexpr float KP = 1.0e-5f;         // 0.1 % por cada 100 cuadros de exceso
  static constexpr float MAXIMO = 0.01f;       // +-1 %
  static constexpr float SUAVE = 0.1f;         // el nivel salta por tramos de 256 (DMA)
  // Devuelve cuantas muestras escribio en 'salida' (capacidad >= n * 1.0101 + 2).
  size_t procesar(const float* x, size_t n, float nivel, float* salida) {
    float e = KP * (nivel - OBJETIVO);
    if (e > MAXIMO) e = MAXIMO;
    if (e < -MAXIMO) e = -MAXIMO;
    e_ += SUAVE * (e - e_);
    const double r = 1.0 + double(e_);
    size_t k = 0;
    for (size_t i = 0; i < n; ++i) {
      const float cur = x[i];
      while (pos_ < 1.0) { salida[k++] = prev_ + (cur - prev_) * float(pos_); pos_ += r; }
      pos_ -= 1.0;
      prev_ = cur;
    }
    return k;
  }
  void reset() { prev_ = 0.0f; pos_ = 0.0; e_ = 0.0f; }
  float ppm() const { return e_ * 1.0e6f; }     // > 0: el Nodo 1 muestrea mas rapido que el DAC
 private:
  float prev_ = 0.0f, e_ = 0.0f;
  double pos_ = 0.0;
};

// Control del reloj del DAC (modo 'reloj apll', el de defecto): en vez de remuestrear el audio se
// ajusta la frecuencia del APLL que mueve el DAC, para que consuma al ritmo con que el Nodo 1 produce.
// El audio sale intacto. e = KP * (nivel suavizado del anillo - objetivo); se reprograma el APLL solo
// cuando e cambia mas de UMBRAL (100 ppm = 0.002 semitonos), asi casi no se toca una vez estable.
class ControlReloj {
 public:
  static constexpr float OBJETIVO = 768.0f;    // cuadros (48 ms); deja lugar a un avance de 512 + el desvio
  static constexpr float KP = 1.0e-5f;         // 0.1 % por cada 100 cuadros de exceso (tau ~6 s)
  static constexpr float MAXIMO = 0.01f;
  static constexpr float SUAVE = 0.02f;        // ~1.6 s con una llamada cada 32 ms
  static constexpr float UMBRAL = 1.0e-4f;
  float actualizar(float nivel) {
    if (primero_) { suave_ = nivel; primero_ = false; }
    suave_ += SUAVE * (nivel - suave_);
    float e = KP * (suave_ - OBJETIVO);
    if (e > MAXIMO) e = MAXIMO;
    if (e < -MAXIMO) e = -MAXIMO;
    return e;
  }
  void reset() { primero_ = true; }
 private:
  float suave_ = OBJETIVO;
  bool primero_ = true;
};

enum class ModoReloj : uint8_t { Apll = 0, Interp = 1, Off = 2 };

struct AudioStats {
  uint32_t escritas = 0;        // bytes entregados al DMA (2 por cuadro)
  uint32_t desbordes = 0;       // cuadros descartados por anillo lleno
  uint32_t vacios = 0;          // veces que el DAC se quedo sin datos (relleno con centro)
  uint32_t muestras_vacias = 0;
  uint32_t saturadas = 0;
  uint32_t errores_dac = 0;
  uint32_t ajustes_apll = 0;    // veces que se reprogramo el APLL
  bool activo = false;
};

#ifndef NODO3_HOST
// Control del DAC. begin() deja la salida en el centro; el DMA arranca solo cuando el anillo
// acumula RING_PREFILL muestras y se detiene con stop().
class AudioOut {
 public:
  bool begin();                              // crea la tarea del DAC y fija el reposo (128)
  // Convierte, satura y encola n cuadros: y25 -> GPIO25, y26 -> GPIO26. Devuelve los cuadros encolados.
  size_t escribir(const float* y25, const float* y26, size_t n);
  size_t cuadros() const { return ring.size() / CANALES_DAC; }   // nivel del anillo en cuadros
  void stop();
  void set_ganancia(float g) { ganancia_ = g; }
  float ganancia() const { return ganancia_; }
  AudioStats stats;
  SampleRing ring;
  AjusteReloj reloj, reloj2;                 // remuestreo (modo 'reloj interp'), uno por canal
  void set_modo_reloj(ModoReloj m);
  ModoReloj modo_reloj() const { return modo_reloj_; }
  float ajuste_ppm() const;                  // > 0: el Nodo 1 va mas rapido que el DAC nominal
  double apll_nominal_hz() const { return f0_; }
 private:
  friend void tarea_dac(void*);
  bool iniciar_dma();
  void detener_dma();
  void* continuo_ = nullptr;                 // dac_continuous_handle_t
  void* reposo_ = nullptr;                   // dac_oneshot_handle_t (GPIO25)
  void* reposo2_ = nullptr;                  // dac_oneshot_handle_t (GPIO26)
  size_t encolar(const uint8_t* c, size_t bytes);
  void reposo();
  volatile bool quiere_ = true;
  float ganancia_ = DAC_GANANCIA;
  void ajustar_apll();
  ModoReloj modo_reloj_ = ModoReloj::Apll;
  ControlReloj control_;
  double f0_ = 0.0;                          // frecuencia del APLL que eligio el driver del DAC
  float e_apll_ = 0.0f;                      // ajuste aplicado al APLL
};
#endif

}  // namespace n3
