#ifndef NODO3_HOST
#include "audio_output.h"
#include <Arduino.h>
#include "driver/dac_continuous.h"
#include "driver/dac_oneshot.h"

namespace n3 {

void tarea_dac(void* arg);
static AudioOut* g_audio = nullptr;

bool AudioOut::begin() {
  g_audio = this;
  dac_oneshot_config_t c = {};
  c.chan_id = DAC_CHAN_0;                    // GPIO25 (DAC1)
  if (dac_oneshot_new_channel(&c, (dac_oneshot_handle_t*)&reposo_) != ESP_OK) return false;
  dac_oneshot_output_voltage((dac_oneshot_handle_t)reposo_, DAC_CENTRO);
  // Nucleo 0 (sin WiFi/BT) para que el DMA se alimente aunque el nucleo 1 este calculando FFT.
  return xTaskCreatePinnedToCore(tarea_dac, "dac", 4096, this, 5, nullptr, 0) == pdPASS;
}

size_t AudioOut::escribir_codigos(const uint8_t* c, size_t n) {
  const size_t k = ring.push(c, n);
  stats.desbordes += uint32_t(n - k);
  return k;
}

size_t AudioOut::escribir(const float* y, size_t n) {
  uint8_t c[DAC_CHUNK];
  size_t total = 0;
  while (n > 0) {
    const size_t m = n < DAC_CHUNK ? n : DAC_CHUNK;
    for (size_t i = 0; i < m; ++i) c[i] = a_codigo_dac(y[i], ganancia_, stats.saturadas);
    total += escribir_codigos(c, m);
    y += m; n -= m;
  }
  return total;
}

bool AudioOut::iniciar_dma() {
  if (reposo_) {                             // el modo directo y el continuo no pueden coexistir
    dac_oneshot_del_channel((dac_oneshot_handle_t)reposo_);
    reposo_ = nullptr;
  }
  dac_continuous_config_t c = {};
  c.chan_mask = DAC_CHANNEL_MASK_CH0;        // GPIO25
  c.desc_num = DAC_DESC_NUM;
  c.buf_size = DAC_DESC_BYTES;
  c.freq_hz = FS;
  c.offset = 0;
  c.clk_src = DAC_DIGI_CLK_SRC_APLL;         // el reloj por defecto no baja de 19.6 kHz
  c.chan_mode = DAC_CHANNEL_MODE_SIMUL;
  if (dac_continuous_new_channels(&c, (dac_continuous_handle_t*)&continuo_) != ESP_OK) { ++stats.errores_dac; return false; }
  if (dac_continuous_enable((dac_continuous_handle_t)continuo_) != ESP_OK) { ++stats.errores_dac; return false; }
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
  dac_oneshot_config_t c = {};
  c.chan_id = DAC_CHAN_0;
  if (dac_oneshot_new_channel(&c, (dac_oneshot_handle_t*)&reposo_) == ESP_OK)
    dac_oneshot_output_voltage((dac_oneshot_handle_t)reposo_, DAC_CENTRO);
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
      ++a->stats.vacios; a->stats.muestras_vacias += uint32_t(DAC_CHUNK - n);
    }
    size_t escritos = 0;
    const esp_err_t e = dac_continuous_write((dac_continuous_handle_t)a->continuo_, buf, DAC_CHUNK, &escritos, 200);
    if (e != ESP_OK) ++a->stats.errores_dac;
    a->stats.escritas += uint32_t(escritos);
  }
}

}  // namespace n3
#endif
