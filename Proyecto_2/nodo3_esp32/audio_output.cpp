#ifndef NODO3_HOST
#include "audio_output.h"
#include <Arduino.h>
#include "driver/dac_continuous.h"
#include "driver/dac_oneshot.h"
#include <math.h>
#include "clk_ctrl_os.h"
#include "hal/clk_tree_ll.h"
#include "esp_private/regi2c_ctrl.h"

namespace n3 {

void tarea_dac(void* arg);
static AudioOut* g_audio = nullptr;

bool AudioOut::begin() {
  g_audio = this;
  reposo();
  if (!reposo_ || !reposo2_) return false;
  // Nucleo 0 (sin WiFi/BT) para que el DMA se alimente aunque el nucleo 1 este calculando FFT.
  return xTaskCreatePinnedToCore(tarea_dac, "dac", 4096, this, 5, nullptr, 0) == pdPASS;
}

// Las dos salidas en reposo (centro) mientras el DMA no corre.
void AudioOut::reposo() {
  dac_oneshot_config_t c = {};
  c.chan_id = DAC_CHAN_0;                    // GPIO25 (DAC1)
  if (dac_oneshot_new_channel(&c, (dac_oneshot_handle_t*)&reposo_) == ESP_OK)
    dac_oneshot_output_voltage((dac_oneshot_handle_t)reposo_, DAC_CENTRO);
  c.chan_id = DAC_CHAN_1;                    // GPIO26 (DAC2)
  if (dac_oneshot_new_channel(&c, (dac_oneshot_handle_t*)&reposo2_) == ESP_OK)
    dac_oneshot_output_voltage((dac_oneshot_handle_t)reposo2_, DAC_CENTRO);
}

// Encola solo cuadros completos: un cuadro partido intercambiaria los canales.
size_t AudioOut::encolar(const uint8_t* c, size_t bytes) {
  size_t libre = ring.libre();
  libre -= libre % CANALES_DAC;
  const size_t k = ring.push(c, bytes < libre ? bytes : libre);
  stats.desbordes += uint32_t((bytes - k) / CANALES_DAC);
  return k / CANALES_DAC;
}

size_t AudioOut::escribir(const float* y25, const float* y26, size_t n) {
  // Tramos de 64 cuadros en la pila (la DRAM estatica esta casi llena). En 'reloj interp' se
  // remuestrean los dos canales con la misma razon (mismo nivel => mismo numero de cuadros).
  constexpr size_t TRAMO = 64;
  constexpr size_t MAXK = TRAMO + TRAMO / 50 + 4;
  float r25[MAXK], r26[MAXK];
  uint8_t c[CANALES_DAC * MAXK];
  size_t total = 0;
  while (n > 0) {
    const size_t m = n < TRAMO ? n : TRAMO;
    size_t k = m;
    const float* a = y25;
    const float* b = y26;
    if (stats.activo && modo_reloj_ == ModoReloj::Interp) {
      const float nivel = float(cuadros());
      k = reloj.procesar(y25, m, nivel, r25);
      const size_t k2 = reloj2.procesar(y26, m, nivel, r26);
      if (k2 < k) k = k2;
      a = r25; b = r26;
    }
    for (size_t i = 0; i < k; ++i) {
      c[CANALES_DAC * i] = a_codigo_dac(a[i], ganancia_, stats.saturadas);
      c[CANALES_DAC * i + 1] = a_codigo_dac(b[i], ganancia_, stats.saturadas);
    }
    total += encolar(c, CANALES_DAC * k);
    y25 += m; y26 += m; n -= m;
  }
  if (stats.activo && modo_reloj_ == ModoReloj::Apll) ajustar_apll();
  return total;
}

// Lee la configuracion del APLL que dejo el driver del DAC: f = xtal (4 + sdm2 + sdm1/256 + sdm0/65536) / (2 (o_div + 2)).
static double apll_actual_hz() {
  uint32_t o = 0, s0 = 0, s1 = 0, s2 = 0;
  clk_ll_apll_get_config(&o, &s0, &s1, &s2);
  return 40.0e6 * (4.0 + s2 + s1 / 256.0 + s0 / 65536.0) / (2.0 * (o + 2));
}

void AudioOut::ajustar_apll() {
  if (f0_ <= 0.0) return;
  const float e = control_.actualizar(float(cuadros()));
  if (fabsf(e - e_apll_) < ControlReloj::UMBRAL) return;
  uint32_t real = 0;
  if (periph_rtc_apll_freq_set(uint32_t(f0_ * (1.0 + e) + 0.5), &real) == ESP_OK) {
    e_apll_ = float(real / f0_ - 1.0);
    ++stats.ajustes_apll;
  } else {                                   // otro periferico usa el APLL: remuestrear
    ++stats.errores_dac;
    modo_reloj_ = ModoReloj::Interp;
  }
}

void AudioOut::set_modo_reloj(ModoReloj m) {
  if (m != ModoReloj::Apll && e_apll_ != 0.0f && f0_ > 0.0) {   // devolver el APLL a su valor nominal
    uint32_t real = 0;
    periph_rtc_apll_freq_set(uint32_t(f0_ + 0.5), &real);
    e_apll_ = 0.0f;
  }
  reloj.reset();
  reloj2.reset();
  control_.reset();
  modo_reloj_ = m;
}

float AudioOut::ajuste_ppm() const {
  if (modo_reloj_ == ModoReloj::Apll) return e_apll_ * 1.0e6f;
  if (modo_reloj_ == ModoReloj::Interp) return reloj.ppm();
  return 0.0f;
}

bool AudioOut::iniciar_dma() {
  if (reposo_) {                             // el modo directo y el continuo no pueden coexistir
    dac_oneshot_del_channel((dac_oneshot_handle_t)reposo_);
    reposo_ = nullptr;
  }
  if (reposo2_) {
    dac_oneshot_del_channel((dac_oneshot_handle_t)reposo2_);
    reposo2_ = nullptr;
  }
  dac_continuous_config_t c = {};
  c.chan_mask = DAC_CHANNEL_MASK_ALL;        // GPIO25 y GPIO26
  c.desc_num = DAC_DESC_NUM;
  c.buf_size = DAC_DESC_BYTES;
  // En modo alternado los datos se reparten A->GPIO25, B->GPIO26, ... y freq_hz es la frecuencia
  // por canal (verificado en placa: el anillo se mantiene estable y los tonos suenan a su altura).
  c.freq_hz = FS;
  c.offset = 0;
  c.clk_src = DAC_DIGI_CLK_SRC_APLL;         // el reloj por defecto no baja de 19.6 kHz
  c.chan_mode = DAC_CHANNEL_MODE_ALTER;
  if (dac_continuous_new_channels(&c, (dac_continuous_handle_t*)&continuo_) != ESP_OK) { ++stats.errores_dac; return false; }
  if (dac_continuous_enable((dac_continuous_handle_t)continuo_) != ESP_OK) { ++stats.errores_dac; return false; }
  reloj.reset();
  reloj2.reset();
  control_.reset();
  f0_ = apll_actual_hz();
  e_apll_ = 0.0f;
  stats.activo = true;
  return true;
}

void AudioOut::detener_dma() {
  if (continuo_) {
    dac_continuous_disable((dac_continuous_handle_t)continuo_);
    dac_continuous_del_channels((dac_continuous_handle_t)continuo_);
    continuo_ = nullptr;
  }
  stats.activo = false;
  reposo();
}

void AudioOut::stop() { quiere_ = false; }

// Tarea del DAC: espera el prefill, arranca el DMA y alimenta bloques fijos desde el anillo.
// dac_continuous_write() bloquea cuando el DMA esta lleno: el ritmo lo impone el reloj APLL.
void tarea_dac(void* arg) {
  AudioOut* a = static_cast<AudioOut*>(arg);
  uint8_t buf[DAC_CHUNK];
  int espera = 0;
  for (;;) {
    if (!a->stats.activo) {
      if (a->quiere_ && a->ring.size() >= RING_PREFILL) a->iniciar_dma();
      else { vTaskDelay(1); continue; }
    }
    if (!a->quiere_) { a->detener_dma(); a->quiere_ = true; a->ring.clear(); continue; }
    // Se espera un bloque completo hasta 40 ms (el DMA conserva 64 ms) antes de declarar un vacio.
    if (a->ring.size() < DAC_CHUNK && espera < 40) { ++espera; vTaskDelay(1); continue; }
    espera = 0;
    size_t n = a->ring.pop(buf, DAC_CHUNK);
    if (n < DAC_CHUNK) {                     // sin datos: relleno con el centro, nunca basura
      memset(buf + n, DAC_CENTRO, DAC_CHUNK - n);
      ++a->stats.vacios; a->stats.muestras_vacias += uint32_t((DAC_CHUNK - n) / CANALES_DAC);
    }
    size_t escritos = 0;
    const esp_err_t e = dac_continuous_write((dac_continuous_handle_t)a->continuo_, buf, DAC_CHUNK, &escritos, 200);
    if (e != ESP_OK) ++a->stats.errores_dac;
    a->stats.escritas += uint32_t(escritos);
  }
}

}  // namespace n3
#endif
