// Nodo 3 - Filtrado espectral adaptativo FFT/IFFT y evaluacion (CE1110, Proyecto Grupal 1).
// Carpeta: Proyecto_2/nodo3_espectral (junto a nodo1_esp32s3 y nodo2_temporal).
// Placa: ESP32 Dev Module (ESP-WROOM-32). Nucleo: Arduino-ESP32 3.3.12. Ver docs/interfaz_nodos.md.
//
//   Nodo 1 --UART2--> [rx task] --cola--> [loop: FFT, filtro, IFFT, OLA] --anillo--> [dac task] --> GPIO25
//
// Estado de verificacion: el DSP, el parser y las metricas estan validados contra Python en el PC
// (nodo3_espectral/tests). El comportamiento en la placa (tiempos, DAC, UART) NO esta medido.
#include <Arduino.h>
#include "esp_arduino_version.h"
#include "esp_heap_caps.h"
#include "config.h"
#include "communication.h"
#include "fft_processing.h"
#include "spectral_filter.h"
#include "audio_output.h"
#include "metrics.h"

#if !CONFIG_IDF_TARGET_ESP32
#error "Seleccione ESP32 Dev Module (ESP-WROOM-32 clasico)."
#endif
#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(3, 3, 12)
#warning "Verificado solo con Arduino-ESP32 3.3.12 (la misma que el emisor)."
#endif
#if !CONFIG_DAC_DMA_AUTO_16BIT_ALIGN
#error "Se requiere CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=1 (buffer uint8_t del DAC continuo)."
#endif

using namespace n3;

// ------------------------------------------------------------------ estado global
static Params g_p;
static SpectralNode g_nodo;
static DiagLocal g_dlocal;
static Diag g_diag;                       // diagnostico activo del bloque en proceso
static Metrics g_met;
static AudioOut g_audio;

struct Bloque { uint16_t seq; int16_t m[N]; };
static Bloque g_pool[POOL_BLOCKS];
static QueueHandle_t g_libres, g_llenos;
static Bloque g_ref[4];                   // referencia limpia (UART1), indexada por seq % 4
static volatile int32_t g_ref_seq[4] = {-1, -1, -1, -1};

static FrameParser g_parser_n1, g_parser_ref;
static SeqTracker g_seq;
static volatile uint32_t g_overflow = 0, g_uart_ovf = 0;
static uint32_t g_bloques = 0, g_perdidos = 0, g_repetidos = 0, g_resync = 0, g_rellenados = 0;

// diagnostico recibido del Nodo 1 (escrito por la tarea RX, leido por loop)
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static DiagFrame g_ext_diag; static PisoFrame g_ext_piso;
static volatile uint32_t g_ext_diag_blk = 0, g_ext_piso_blk = 0;
static volatile bool g_ext_diag_ok = false, g_ext_piso_ok = false;
static volatile uint32_t g_blk_actual = 0, g_ext_fallback = 0;

static float g_dc_in = 0;                 // media de la ultima entrada (ya escalada): debe ser ~0
static bool g_ref_activa = false, g_stream = false, g_fuente_interna = false;
static uint32_t g_prox_estado_ms = 0;

// generador interno (pruebas sin Nodo 1): dos tonos + ruido blanco
static float g_gen_f1 = 1000, g_gen_a1 = 0.4f, g_gen_f2 = 2500, g_gen_a2 = 0.2f, g_gen_ruido = 0.0f;
static double g_gen_fase1 = 0, g_gen_fase2 = 0;
static uint32_t g_gen_rng = 12345;
static uint16_t g_gen_seq = 0;
static uint32_t g_gen_proximo_us = 0;

// ------------------------------------------------------------------ recepcion (tarea RX)
struct SinkN1 : FrameParser::Sink {
  void on_pcm(uint16_t seq, const int16_t* m) override {
    int idx;
    if (xQueueReceive(g_libres, &idx, 0) != pdTRUE) { g_overflow = g_overflow + 1; return; }   // cola llena: se descarta el nuevo
    g_pool[idx].seq = seq;
    memcpy(g_pool[idx].m, m, sizeof g_pool[idx].m);
    xQueueSend(g_llenos, &idx, 0);
  }
  void on_diag(const DiagFrame& f) override {
    portENTER_CRITICAL(&g_mux); g_ext_diag = f; g_ext_diag_blk = g_blk_actual; g_ext_diag_ok = true; portEXIT_CRITICAL(&g_mux);
  }
  void on_piso(const PisoFrame& f) override {
    portENTER_CRITICAL(&g_mux); g_ext_piso = f; g_ext_piso_blk = g_blk_actual; g_ext_piso_ok = true; portEXIT_CRITICAL(&g_mux);
  }
};
struct SinkRef : FrameParser::Sink {
  void on_pcm(uint16_t seq, const int16_t* m) override {
    const int s = seq & 3;
    g_ref_seq[s] = -1;
    memcpy(g_ref[s].m, m, sizeof g_ref[s].m);
    g_ref[s].seq = seq;
    g_ref_seq[s] = seq;
  }
};
static SinkN1 g_sink_n1;
static SinkRef g_sink_ref;

static void tarea_rx(void*) {
  uint8_t buf[256];
  for (;;) {
    bool hubo = false;
    int n = Serial2.available();
    if (n > 0) { n = Serial2.read(buf, n < 256 ? n : 256); if (n > 0) g_parser_n1.feed(buf, n, g_sink_n1); hubo = true; }
    if (g_ref_activa) {
      n = Serial1.available();
      if (n > 0) { n = Serial1.read(buf, n < 256 ? n : 256); if (n > 0) g_parser_ref.feed(buf, n, g_sink_ref); hubo = true; }
    }
    if (!hubo) vTaskDelay(1);
  }
}

// ------------------------------------------------------------------ procesamiento
static float g_x[N], g_y[N], g_s[N];
static float g_prev_x[HOP], g_prev_s[HOP], g_raw_prev[HOP];
static bool g_tiene_prev = false;

// Sustituye el piso/interferentes locales por los del Nodo 1 cuando estan frescos (< 5 bloques).
static void elegir_diagnostico(const float* x) {
  bool piso_ext = false, interf_ext = false;
  DiagFrame par{}; PisoFrame piso{};
  if (g_p.fuente == FuenteDiag::Externo) {
    portENTER_CRITICAL(&g_mux);
    const bool pf = g_ext_piso_ok && g_blk_actual - g_ext_piso_blk < 5;
    const bool qf = g_ext_diag_ok && g_blk_actual - g_ext_diag_blk < 5;
    if (pf) piso = g_ext_piso;
    if (qf) par = g_ext_diag;
    portEXIT_CRITICAL(&g_mux);
    piso_ext = pf; interf_ext = qf;
  }
  if (!(piso_ext && interf_ext)) {
    if (g_p.fuente == FuenteDiag::Externo) g_ext_fallback = g_ext_fallback + 1;
    g_dlocal.process(x, g_p, g_diag);          // estimador propio (replica del Nodo 1)
  }
  if (piso_ext) diag_set_piso_knots(g_diag, piso.centi_db, piso.nk, piso.decim);
  if (interf_ext) {
    float f[MAX_FREQ]; uint8_t fl[MAX_FREQ];
    for (int i = 0; i < par.n; ++i) { f[i] = par.f[i].f_hz; fl[i] = par.f[i].flags; }
    diag_set_interf_hz(g_diag, f, fl, par.n);
  }
}

// Procesa un bloque de N muestras (float) y entrega N muestras a la salida.
static void procesar_float(const float* x, const float* s /* referencia o nullptr */) {
  elegir_diagnostico(x);
  for (int h = 0; h < 2; ++h) {
    HopStats hs;
    float* y = g_y + h * HOP;
    g_nodo.advance(x + h * HOP, &g_diag, g_p, y, &hs);
    if (g_p.modo == Modo::Crudo) memcpy(y, g_raw_prev, sizeof g_raw_prev);   // entrada retardada HOP: A/B justo
    memcpy(g_raw_prev, x + h * HOP, sizeof g_raw_prev);
    g_audio.escribir(y, HOP);
    g_met.on_hop(hs);
    if (s && g_tiene_prev) g_met.on_samples(g_prev_s, g_prev_x, y, HOP);      // y[t] corresponde a x[t-HOP]
    memcpy(g_prev_x, x + h * HOP, sizeof g_prev_x);
    if (s) memcpy(g_prev_s, s + h * HOP, sizeof g_prev_s);
    g_tiene_prev = true;
  }
}

static void procesar_bloque(const Bloque& b) {
  const uint32_t t0 = micros();
  const int perdidos = g_seq.on_seq(b.seq, 65536);
  if (perdidos < 0) { ++g_repetidos; return; }                   // repetido / fuera de orden
  if (perdidos > 0) {
    g_perdidos += perdidos;
    if (perdidos > MAX_FILL_BLOCKS) {                            // salto grande: reiniciar historia
      ++g_resync; g_nodo.reset(); g_dlocal.reset(); g_tiene_prev = false;
    } else {                                                     // huecos cortos: ceros para mantener el reloj
      memset(g_x, 0, sizeof g_x);
      for (int i = 0; i < perdidos; ++i) { procesar_float(g_x, nullptr); ++g_rellenados; }
    }
  }
  double suma = 0;
  for (int i = 0; i < N; ++i) { g_x[i] = muestra_a_float(b.m[i], g_p.formato, g_p.adc_mid); suma += g_x[i]; }
  g_dc_in = float(suma / N);
  const float* ref = nullptr;
  if (g_ref_activa && g_ref_seq[b.seq & 3] == b.seq) {
    for (int i = 0; i < N; ++i) g_s[i] = g_ref[b.seq & 3].m[i] * (1.0f / 32768.0f);   // la referencia es siempre Q15
    ref = g_s;
  }
  procesar_float(g_x, ref);
  ++g_bloques; g_blk_actual = g_blk_actual + 1;
  g_met.on_block_time(micros() - t0);
}

// ------------------------------------------------------------------ generador interno
static float ruido_blanco() {                                    // xorshift32 -> [-1, 1)
  g_gen_rng ^= g_gen_rng << 13; g_gen_rng ^= g_gen_rng >> 17; g_gen_rng ^= g_gen_rng << 5;
  return (g_gen_rng >> 8) * (2.0f / 16777216.0f) - 1.0f;
}
static void generar_bloque() {
  static Bloque b;
  const double w1 = 2 * M_PI * g_gen_f1 / FS, w2 = 2 * M_PI * g_gen_f2 / FS;
  for (int i = 0; i < N; ++i) {
    float v = g_gen_a1 * sin(g_gen_fase1) + g_gen_a2 * sin(g_gen_fase2) + g_gen_ruido * ruido_blanco();
    g_gen_fase1 += w1; if (g_gen_fase1 > 2 * M_PI) g_gen_fase1 -= 2 * M_PI;
    g_gen_fase2 += w2; if (g_gen_fase2 > 2 * M_PI) g_gen_fase2 -= 2 * M_PI;
    if (v > 0.999f) v = 0.999f; else if (v < -0.999f) v = -0.999f;
    b.m[i] = int16_t(lrintf(v * 32768.0f));
  }
  b.seq = g_gen_seq++;
  procesar_bloque(b);
}

// ------------------------------------------------------------------ consola
static void imprimir_parametros() {
  Serial.printf("alfa=%.3f beta=%.3f ginterf=%.4f notch=%d lambda=%.2f banda=%.0f-%.0f Hz rint=%.0f-%.0f Hz modo=%s diag=%s ref=%s fuente=%s entrada=%s(mid=%.0f)\n",
                g_p.alfa, g_p.beta, g_p.g_interf, g_p.notch_bins, g_p.lambda_ema, g_p.banda_lo, g_p.banda_hi,
                g_p.rint_lo, g_p.rint_hi, g_p.modo == Modo::Filtro ? "filtro" : g_p.modo == Modo::Bypass ? "bypass" : "crudo",
                g_p.fuente == FuenteDiag::Local ? "local" : "ext", g_ref_activa ? "on" : "off",
                g_fuente_interna ? "generador" : "uart", g_p.formato == FormatoIn::Adc12 ? "adc12" : "q15", g_p.adc_mid);
}

static void imprimir_metricas() {
  const Reporte r = g_met.reporte();
  Serial.printf("[SIN REFERENCIA] hops=%lu atenuacion_rango=%.2f dB util_conservada=%.1f %% reduccion_piso(est)=%.2f dB "
                "SNR_est_in=%.2f dB SNR_est_out=%.2f dB (ESTIMADAS, no son SNR verdaderos) bins_atenuados=%.1f\n",
                (unsigned long)r.hops, r.atenuacion_rng_db, r.util_conservada_pct, r.reduccion_piso_db,
                r.snr_est_in_db, r.snr_est_out_db, r.bins_atenuados_medio);
  if (r.ref_muestras) {
    Serial.printf("[CON REFERENCIA] muestras=%lu MSE=%.4e SNR_in=%.2f dB SNR_out=%.2f dB dSNR=%.2f dB E_util=%.1f %%\n",
                  (unsigned long)r.ref_muestras, r.mse, r.snr_in, r.snr_out, r.dsnr, r.e_util);
  } else {
    Serial.println("[CON REFERENCIA] no disponible: active 'ref on', conecte la referencia limpia y use 'metrics reset' antes de la prueba.");
  }
  const double avg = g_met.t_blocks ? g_met.t_sum_us / g_met.t_blocks : 0;
  Serial.printf("[TIEMPO] bloques=%lu us/bloque: prom=%.0f min=%lu max=%lu (presupuesto %lu us) excedidos=%lu\n",
                (unsigned long)g_met.t_blocks, avg, (unsigned long)(g_met.t_blocks ? g_met.t_min_us : 0),
                (unsigned long)g_met.t_max_us, (unsigned long)(1000000ull * N / FS), (unsigned long)g_met.t_excedidos);
}

static void imprimir_estado() {
  const LinkStats& a = g_parser_n1.stats();
  Serial.printf("[ENLACE N1] bytes=%lu pcm_ok=%lu diag=%lu piso=%lu crc_err=%lu bytes_basura=%lu\n",
                (unsigned long)a.bytes, (unsigned long)a.pcm_ok, (unsigned long)a.diag_ok, (unsigned long)a.piso_ok,
                (unsigned long)a.crc_err, (unsigned long)a.bytes_descartados);
  Serial.printf("[ENTRADA] DC de la ultima trama = %+.4f (escala completa 1.0; un DC alto desplaza la salida del DAC; ajuste 'set adcmid')\n", g_dc_in);
  Serial.printf("[BLOQUES] procesados=%lu perdidos=%lu rellenados=%lu repetidos=%lu resync=%lu desbordes_cola=%lu\n",
                (unsigned long)g_bloques, (unsigned long)g_perdidos, (unsigned long)g_rellenados,
                (unsigned long)g_repetidos, (unsigned long)g_resync, (unsigned long)g_overflow);
  Serial.printf("[DAC] activo=%d escritas=%lu vacios=%lu (muestras %lu) desbordes_anillo=%lu saturadas=%lu errores=%lu anillo=%u/%u (%.0f ms)\n",
                g_audio.stats.activo, (unsigned long)g_audio.stats.escritas, (unsigned long)g_audio.stats.vacios,
                (unsigned long)g_audio.stats.muestras_vacias, (unsigned long)g_audio.stats.desbordes,
                (unsigned long)g_audio.stats.saturadas, (unsigned long)g_audio.stats.errores_dac,
                (unsigned)g_audio.ring.size(), (unsigned)RING_SAMPLES, 1000.0 * g_audio.ring.size() / FS);
  Serial.printf("[MEMORIA] heap libre=%lu minimo=%lu total=%lu B; pila loop libre=%u B\n",
                (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getHeapSize(),
                (unsigned)uxTaskGetStackHighWaterMark(nullptr));
  Serial.printf("[LATENCIA] algoritmica=%lu ms (bloque %lu + solape %lu) + DMA %lu ms + anillo %.0f ms; "
                "fallbacks de diagnostico externo=%lu\n",
                (unsigned long)(1000ul * (N + HOP) / FS), (unsigned long)(1000ul * N / FS), (unsigned long)(1000ul * HOP / FS),
                (unsigned long)(1000ul * DAC_DESC_NUM * DAC_DESC_BYTES / 2 / FS), 1000.0 * g_audio.ring.size() / FS,
                (unsigned long)g_ext_fallback);
}

static void benchmark() {
  static Diag d; static float x[N], y[HOP];
  Fft fft; fft.begin();
  alignas(16) static float c[2 * N];
  for (int i = 0; i < N; ++i) x[i] = 0.5f * sinf(2 * M_PI * 1000.0f * i / FS) + 0.2f * ruido_blanco();
  const int REP = 50;
  uint32_t t_fft = 0, t_max_fft = 0, t_diag = 0, t_adv = 0, t_blk = 0, t_max_blk = 0;
  for (int r = 0; r < REP; ++r) {
    for (int i = 0; i < N; ++i) { c[2 * i] = x[i]; c[2 * i + 1] = 0; }
    uint32_t t = micros(); fft.forward(c); fft.inverse(c); t = micros() - t;
    t_fft += t; if (t > t_max_fft) t_max_fft = t;
  }
  for (int r = 0; r < REP; ++r) { uint32_t t = micros(); g_dlocal.process(x, g_p, d); t_diag += micros() - t; }
  for (int r = 0; r < REP; ++r) { uint32_t t = micros(); g_nodo.advance(x, &d, g_p, y); t_adv += micros() - t; }
  for (int r = 0; r < REP; ++r) {
    uint32_t t = micros(); g_dlocal.process(x, g_p, d);
    g_nodo.advance(x, &d, g_p, y); g_nodo.advance(x + HOP, &d, g_p, y);
    t = micros() - t; t_blk += t; if (t > t_max_blk) t_max_blk = t;
  }
  g_nodo.reset(); g_dlocal.reset();
  Serial.printf("[BENCH %d repeticiones, sobre senal sintetica, sin UART ni DAC]\n", REP);
  Serial.printf("  FFT+IFFT 1024 pts (%s): prom=%lu us, max=%lu us\n", fft.backend(), (unsigned long)(t_fft / REP), (unsigned long)t_max_fft);
  Serial.printf("  diagnostico local: prom=%lu us\n  avance del filtro (1 hop): prom=%lu us\n", (unsigned long)(t_diag / REP), (unsigned long)(t_adv / REP));
  Serial.printf("  bloque completo (diag + 2 avances): prom=%lu us, max=%lu us => %.1f %% del presupuesto de %lu us\n",
                (unsigned long)(t_blk / REP), (unsigned long)t_max_blk, 100.0 * (t_blk / REP) / (1000000.0 * N / FS),
                (unsigned long)(1000000ull * N / FS));
}

static void escenario(const char* id) {
  struct E { const char* id; float blo, bhi, rlo, rhi; };
  static const E t[] = {{"P1", 900, 1100, 2485, 2515}, {"P2", 340, 1420, 4500, 7500}, {"P3", 150, 2700, 3400, 8000},
                        {"P4", 300, 900, 1600, 8000}, {"P5", 340, 1420, 1000, 2200}};
  for (const E& e : t)
    if (!strcasecmp(id, e.id)) {
      g_p.banda_lo = e.blo; g_p.banda_hi = e.bhi; g_p.rint_lo = e.rlo; g_p.rint_hi = e.rhi;
      g_met.reset(); g_tiene_prev = false;
      Serial.printf("Escenario %s: banda util %.0f-%.0f Hz, rango de interferencia %.0f-%.0f Hz. Metricas reiniciadas.\n", e.id, e.blo, e.bhi, e.rlo, e.rhi);
      return;
    }
  Serial.println("P6 (desconocido): fije 'set band lo hi' y 'set rint lo hi' segun la prueba. IDs validos: P1..P5.");
}

static void ayuda() {
  Serial.println(
    "Comandos (terminar con Enter):\n"
    "  status | params | metrics [reset] | bench | stream on|off\n"
    "  set alfa|beta|ginterf|lambda <v> | set notch <bins> | set band <lo> <hi> | set rint <lo> <hi>\n"
    "  mode filter|bypass|raw     filter=sustraccion espectral, bypass=FFT+IFFT con G=1, raw=entrada retardada HOP\n"
    "  diag local|ext             origen del piso/interferentes (ext = tramas del Nodo 1, con respaldo local)\n"
    "  scen P1..P5                banda util y rango de interferencia del escenario\n"
    "  ref on|off                 referencia limpia por UART1 (metricas verdaderas); 'metrics reset' antes de la prueba\n"
 "  fmt adc12|q15 | set adcmid <cuentas>   formato de la trama PCM de entrada (adc12 = cuenta cruda del Nodo 1, centro 2048)\n"
    "  baud <n>                   velocidad de la UART2 (debe coincidir con el Nodo 1; defecto 921600)\n"
    "  src uart|gen               fuente: UART2 (Nodo 1) o generador interno\n"
    "  gen f1 a1 f2 a2 ruido      generador: dos tonos (Hz, amplitud 0-1) + ruido blanco (amplitud)\n"
    "  gain <codigos>             ganancia del DAC (60 = igual que el Nodo 2; 127 = escala completa), dac stop|start");
}

static void comando(char* l) {
  char* a = strtok(l, " \t");
  if (!a) return;
  char* b = strtok(nullptr, " \t"); char* c = strtok(nullptr, " \t"); char* d = strtok(nullptr, " \t");
  char* e = strtok(nullptr, " \t"); char* f = strtok(nullptr, " \t");
  if (!strcmp(a, "help") || !strcmp(a, "?")) ayuda();
  else if (!strcmp(a, "status")) imprimir_estado();
  else if (!strcmp(a, "params")) imprimir_parametros();
  else if (!strcmp(a, "metrics")) { if (b && !strcmp(b, "reset")) { g_met.reset(); g_tiene_prev = false; Serial.println("metricas reiniciadas"); } else imprimir_metricas(); }
  else if (!strcmp(a, "bench")) benchmark();
  else if (!strcmp(a, "stream")) {
    g_stream = b && !strcmp(b, "on");
    if (g_stream) Serial.println("#N3,ms,bloques,perdidos,crc,desb_cola,vacios_dac,saturadas,us_prom,us_max,aten_db,util_pct,snr_est_in,snr_est_out,anillo,mse,snr_in,snr_out,dsnr,e_util");
  }
  else if (!strcmp(a, "set") && b && c) {
    const float v = atof(c);
    if (!strcmp(b, "alfa")) g_p.alfa = v;
    else if (!strcmp(b, "beta")) g_p.beta = constrain(v, 0.0f, 1.0f);
    else if (!strcmp(b, "ginterf")) g_p.g_interf = constrain(v, 0.0f, 1.0f);
    else if (!strcmp(b, "lambda")) g_p.lambda_ema = constrain(v, 0.0f, 0.999f);
    else if (!strcmp(b, "notch")) g_p.notch_bins = constrain(int(v), 0, 32);
    else if (!strcmp(b, "band") && d) { g_p.banda_lo = v; g_p.banda_hi = atof(d); }
    else if (!strcmp(b, "adcmid")) g_p.adc_mid = constrain(v, 0.0f, 4095.0f);
    else if (!strcmp(b, "rint") && d) { g_p.rint_lo = v; g_p.rint_hi = atof(d); }
    else { Serial.println("parametro desconocido"); return; }
    imprimir_parametros();
  }
  else if (!strcmp(a, "mode") && b) {
    g_p.modo = !strcmp(b, "bypass") ? Modo::Bypass : !strcmp(b, "raw") ? Modo::Crudo : Modo::Filtro; imprimir_parametros();
  }
  else if (!strcmp(a, "diag") && b) { g_p.fuente = !strcmp(b, "ext") ? FuenteDiag::Externo : FuenteDiag::Local; imprimir_parametros(); }
  else if (!strcmp(a, "scen") && b) escenario(b);
  else if (!strcmp(a, "fmt") && b) { g_p.formato = !strcmp(b, "q15") ? FormatoIn::Q15 : FormatoIn::Adc12; imprimir_parametros(); }
  else if (!strcmp(a, "baud") && b) {
    const unsigned long v = strtoul(b, nullptr, 10);
    if (v >= 9600 && v <= 2000000) { Serial2.updateBaudRate(v); if (g_ref_activa) Serial1.updateBaudRate(v); Serial.printf("UART a %lu baud\n", v); }
    else Serial.println("baud fuera de rango (9600..2000000)");
  }
  else if (!strcmp(a, "ref") && b) {
    g_ref_activa = !strcmp(b, "on");
    if (g_ref_activa) { Serial1.setRxBufferSize(8192); Serial1.begin(BAUD, SERIAL_8N1, PIN_RX_REF, PIN_TX_REF); }   // GPIO18/19 solo se usan con la referencia
    else Serial1.end();
    g_met.reset(); g_tiene_prev = false; imprimir_parametros(); }
  else if (!strcmp(a, "src") && b) { g_fuente_interna = !strcmp(b, "gen"); g_gen_proximo_us = micros(); imprimir_parametros(); }
  else if (!strcmp(a, "gen") && b) {
    g_gen_f1 = atof(b); if (c) g_gen_a1 = atof(c); if (d) g_gen_f2 = atof(d); if (e) g_gen_a2 = atof(e); if (f) g_gen_ruido = atof(f);
    Serial.printf("generador: %.0f Hz x%.2f + %.0f Hz x%.2f + ruido %.2f\n", g_gen_f1, g_gen_a1, g_gen_f2, g_gen_a2, g_gen_ruido);
  }
  else if (!strcmp(a, "gain") && b) { g_audio.set_ganancia(constrain(atof(b), 0.0f, 127.0f)); Serial.printf("ganancia DAC = %.1f codigos\n", g_audio.ganancia()); }
  else if (!strcmp(a, "dac") && b) { if (!strcmp(b, "stop")) g_audio.stop(); Serial.println("dac: el DMA arranca solo al acumular el prefill"); }
  else Serial.println("comando desconocido; escriba help");
}

static void atender_consola() {
  static char linea[96]; static size_t n = 0;
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\r' || ch == '\n') { if (n) { linea[n] = 0; comando(linea); n = 0; } }
    else if (n < sizeof linea - 1) linea[n++] = ch;
  }
}

static void emitir_estado_periodico() {
  if (!g_stream || millis() < g_prox_estado_ms) return;
  g_prox_estado_ms = millis() + 1000;
  const Reporte r = g_met.reporte();
  const double avg = g_met.t_blocks ? g_met.t_sum_us / g_met.t_blocks : 0;
  Serial.printf("N3,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.0f,%lu,%.2f,%.1f,%.2f,%.2f,%u,%.4e,%.2f,%.2f,%.2f,%.1f\n",
                (unsigned long)millis(), (unsigned long)g_bloques, (unsigned long)g_perdidos,
                (unsigned long)g_parser_n1.stats().crc_err, (unsigned long)g_overflow,
                (unsigned long)g_audio.stats.vacios, (unsigned long)g_audio.stats.saturadas, avg,
                (unsigned long)g_met.t_max_us, r.atenuacion_rng_db, r.util_conservada_pct, r.snr_est_in_db,
                r.snr_est_out_db, (unsigned)g_audio.ring.size(), r.mse, r.snr_in, r.snr_out, r.dsnr, r.e_util);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nNodo 3 - filtrado espectral FFT/IFFT (CE1110)");
  if (!g_nodo.begin() || !g_dlocal.begin()) { Serial.println("ERROR: no se pudo iniciar la FFT"); for (;;) delay(1000); }
  g_diag.piso_valido = false; g_diag.n_interf = 0;

  g_libres = xQueueCreate(POOL_BLOCKS, sizeof(int));
  g_llenos = xQueueCreate(POOL_BLOCKS, sizeof(int));
  for (int i = 0; i < POOL_BLOCKS; ++i) xQueueSend(g_libres, &i, 0);

  Serial2.setRxBufferSize(8192);                       // >= 3 tramas PCM mientras se calcula la FFT
  Serial2.begin(BAUD, SERIAL_8N1, PIN_RX_NODO1, PIN_TX_NODO1);
  xTaskCreatePinnedToCore(tarea_rx, "rx", 6144, nullptr, 4, nullptr, 0);

  if (!g_audio.begin()) Serial.println("ERROR: no se pudo iniciar el DAC");
  Serial.printf("FFT: %s | UART2 RX=GPIO%d a %lu baud | DAC: GPIO25 (8 bits, %lu Hz, APLL)\n", g_nodo.backend(),
                PIN_RX_NODO1, (unsigned long)BAUD, (unsigned long)FS);
  Serial.printf("Heap libre tras iniciar: %lu B. Escriba 'help'.\n", (unsigned long)ESP.getFreeHeap());
  imprimir_parametros();
}

void loop() {
  int idx;
  if (!g_fuente_interna) {
    if (xQueueReceive(g_llenos, &idx, pdMS_TO_TICKS(2)) == pdTRUE) {
      procesar_bloque(g_pool[idx]);
      xQueueSend(g_libres, &idx, 0);
    }
  } else {
    const uint32_t ahora = micros();
    if (int32_t(ahora - g_gen_proximo_us) >= 0) { g_gen_proximo_us += 1000000ull * N / FS; generar_bloque(); }
    else vTaskDelay(1);
  }
  atender_consola();
  emitir_estado_periodico();
}
