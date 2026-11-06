// Nodo 3 - Filtrado espectral adaptativo FFT/IFFT y evaluacion (CE1110, Proyecto Grupal 1).
// Placa: ESP32 Dev Module (ESP-WROOM-32). Nucleo: Arduino-ESP32 3.3.12. Ver README.md.
//
//   Nodo 1 --UART2--> [rx task] --cola--> [loop: FFT, filtro, IFFT, OLA] --anillo--> [dac task] --> GPIO25
//
// Un solo parlante: 'mode filter' (espectral, este nodo), 'mode fir' (temporal, metodo del Nodo 2),
// 'mode raw' (entrada sin filtrar) eligen que se escucha. 'scen P1..P5' elige tambien el perfil FIR.
//
// Estado de verificacion: el DSP, el parser y las metricas estan validados contra Python en el PC
// (nodo3_esp32/tests). El comportamiento en la placa (tiempos, DAC, UART) NO esta medido.
#include <Arduino.h>
#include "esp_arduino_version.h"
#include "esp_heap_caps.h"
#include "config.h"
#include "communication.h"
#include "fft_processing.h"
#include "spectral_filter.h"
#include "audio_output.h"
#include "metrics.h"
#include "salida_fir.h"
#include "recomendacion.h"
#include "captura.h"
#include "pasa_altos.h"
#include <Preferences.h>
#include <new>

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
// En el heap (setup): como globales no caben en la DRAM estatica del ESP32 clasico.
static SalidaFir* g_fir = nullptr;        // filtro temporal (Nodo 2) para 'mode fir'
static StatsSenales* g_stats_fir = nullptr;   // metricas espectrales de la salida FIR
static Metrics* g_met_fir = nullptr;      // metricas de la ruta temporal (Nodo 2)
static PasaAltos g_hpf;                   // acondicionamiento de entrada ('hpf <Hz>')
constexpr float HPF_DEFECTO_HZ = 0.0f;   // el pasa altos va en el Nodo 1 (h100): aqui apagado para no filtrar dos veces

// 'captura': graba un tramo de entrada + las dos salidas y lo envia por USB sin bloquear el proceso.
enum class EstadoCaptura : uint8_t { Libre, Grabando, Enviando };
static EstadoCaptura g_cap_estado = EstadoCaptura::Libre;
static uint8_t* g_cap = nullptr;          // la trama completa, en el heap solo mientras se usa
static size_t g_cap_n = 0, g_cap_idx = 0, g_cap_largo = 0, g_cap_enviado = 0;

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
static ParamsFrame g_ext_par; static PisoFrame g_ext_piso;
static volatile uint32_t g_ext_par_blk = 0, g_ext_piso_blk = 0;
static volatile bool g_ext_par_ok = false, g_ext_piso_ok = false;
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
  void on_params(const ParamsFrame& f) override {
    portENTER_CRITICAL(&g_mux); g_ext_par = f; g_ext_par_blk = g_blk_actual; g_ext_par_ok = true; portEXIT_CRITICAL(&g_mux);
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
  ParamsFrame par{}; PisoFrame piso{};
  // Con el generador interno el diagnostico del Nodo 1 describe el microfono, no esta senal:
  // se usa siempre el local (si no, el parlante, el microfono y el filtro forman un lazo).
  const bool usar_ext = g_p.fuente == FuenteDiag::Externo && !g_fuente_interna;
  if (usar_ext) {
    portENTER_CRITICAL(&g_mux);
    const bool pf = g_ext_piso_ok && g_blk_actual - g_ext_piso_blk < 5;
    const bool qf = g_ext_par_ok && g_blk_actual - g_ext_par_blk < 5;
    if (pf) piso = g_ext_piso;
    if (qf) par = g_ext_par;
    portEXIT_CRITICAL(&g_mux);
    piso_ext = pf; interf_ext = qf;
  }
  if (!(piso_ext && interf_ext)) {
    if (usar_ext) g_ext_fallback = g_ext_fallback + 1;
    g_dlocal.process(x, g_p, g_diag);          // estimador propio (replica del Nodo 1)
  }
  if (piso_ext) diag_set_piso_knots(g_diag, piso.centi_db, piso.nk, piso.decim);
  if (interf_ext) {
    float f[MAX_FREQ]; uint8_t fl[MAX_FREQ];
    for (int i = 0; i < par.n; ++i) { f[i] = par.f[i].f_hz; fl[i] = par.f[i].flags; }
    diag_set_interf_hz(g_diag, f, fl, par.n);
  }
}

// Procesa un bloque de N muestras (float) por las DOS rutas independientes, en cada avance de HOP:
//   ruta espectral (Nodo 3): FFT, G, IFFT, OLA   -> y_esp, metricas g_met
//   ruta temporal  (Nodo 2): FIR de fase lineal  -> y_fir, metricas g_met_fir
// Las dos salen retardadas exactamente HOP (comparables muestra a muestra con la entrada).
// GPIO25 saca lo que elige 'mode' (filter/fir/raw/bypass); GPIO26 saca siempre el FIR.
static void procesar_float(const float* x, const float* s /* referencia o nullptr */) {
  elegir_diagnostico(x);
  float* y_esp = g_y;                                // dos mitades de g_y: sin mas memoria estatica
  float* y_fir = g_y + HOP;
  for (int h = 0; h < 2; ++h) {
    HopStats hs, hf;
    g_nodo.advance(x + h * HOP, &g_diag, g_p, y_esp, &hs);
    g_fir->procesar(x + h * HOP, y_fir);
    g_stats_fir->on_hop(g_raw_prev, y_fir, &g_diag, g_p, &hf);
    const float* y25 = g_p.modo == Modo::Fir ? y_fir : g_p.modo == Modo::Crudo ? g_raw_prev : y_esp;
    g_audio.escribir(y25, y_fir, HOP);
    g_met.on_hop(hs);
    g_met_fir->on_hop(hf);
    if (s && g_tiene_prev) {                                                 // y[t] corresponde a x[t-HOP]
      g_met.on_samples(g_prev_s, g_prev_x, y_esp, HOP);
      g_met_fir->on_samples(g_prev_s, g_prev_x, y_fir, HOP);
    }
    if (g_cap_estado == EstadoCaptura::Grabando) {                         // entrada alineada con las salidas
      captura_muestras(g_cap, g_cap_n, g_cap_idx, g_raw_prev, y_esp, y_fir, HOP);
      g_cap_idx += HOP;
      if (g_cap_idx >= g_cap_n) {
        g_cap_largo = captura_cerrar(g_cap, g_cap_n, uint8_t(g_p.modo), uint8_t(g_fir->perfil()), g_diag, g_p);
        g_cap_enviado = 0;
        g_cap_estado = EstadoCaptura::Enviando;
      }
    }
    memcpy(g_raw_prev, x + h * HOP, sizeof g_raw_prev);
    memcpy(g_prev_x, x + h * HOP, sizeof g_prev_x);
    if (s) memcpy(g_prev_s, s + h * HOP, sizeof g_prev_s);
    g_tiene_prev = true;
  }
}

// Reinicia la historia del proceso (secuencia, filtros, alineacion) tras un corte de la fuente.
static void resincronizar() {
  g_seq.reset();
  g_nodo.reset(); g_dlocal.reset(); g_fir->reiniciar(); g_stats_fir->reset(); g_tiene_prev = false;
  g_hpf.reset();
  ++g_resync;
}

static void procesar_bloque(const Bloque& b) {
  const uint32_t t0 = micros();
  static uint32_t repetidos_seguidos = 0;
  int perdidos = g_seq.on_seq(b.seq, 65536);
  if (perdidos < 0) {                                            // repetido / fuera de orden
    ++g_repetidos;
    // Varios "viejos" seguidos no son duplicados: la fuente se reinicio (Nodo 1 recargado o cambio de
    // 'src') y volvio a contar desde 0. Sin esto se descartaria todo hasta dar la vuelta (70 min).
    if (++repetidos_seguidos < 3) return;
    resincronizar();
    perdidos = g_seq.on_seq(b.seq, 65536);
  }
  repetidos_seguidos = 0;
  if (perdidos > 0) {
    g_perdidos += perdidos;
    if (perdidos > MAX_FILL_BLOCKS) {                            // salto grande: reiniciar historia
      ++g_resync; g_nodo.reset(); g_dlocal.reset(); g_fir->reiniciar(); g_stats_fir->reset(); g_tiene_prev = false;
    } else {                                                     // huecos cortos: ceros para mantener el reloj
      memset(g_x, 0, sizeof g_x);
      for (int i = 0; i < perdidos; ++i) { procesar_float(g_x, nullptr); ++g_rellenados; }
    }
  }
  double suma = 0;
  for (int i = 0; i < N; ++i) { g_x[i] = muestra_a_float(b.m[i], g_p.formato, g_p.adc_mid); suma += g_x[i]; }
  g_dc_in = float(suma / N);                                     // antes del pasa altos: sirve para ajustar adcmid
  g_hpf.procesar(g_x, N);                                        // quita DC y graves (< hpf Hz) antes de las dos rutas
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
    // en el formato de entrada activo, para que 'src gen' no dependa de 'fmt'
    b.m[i] = g_p.formato == FormatoIn::Q15 ? int16_t(lrintf(v * 32768.0f))
                                           : int16_t(lrintf(v * 2048.0f + g_p.adc_mid));
  }
  b.seq = g_gen_seq++;
  procesar_bloque(b);
}

// ------------------------------------------------------------------ consola
static void imprimir_parametros() {
  Serial.printf("alfa=%.3f beta=%.3f ginterf=%.4f notch=%d lambda=%.2f banda=%.0f-%.0f Hz rint=%.0f-%.0f Hz modo=%s fir=%s hpf=%.0f diag=%s ref=%s fuente=%s entrada=%s(mid=%.0f)\n",
                g_p.alfa, g_p.beta, g_p.g_interf, g_p.notch_bins, g_p.lambda_ema, g_p.banda_lo, g_p.banda_hi,
                g_p.rint_lo, g_p.rint_hi,
                g_p.modo == Modo::Filtro ? "filtro" : g_p.modo == Modo::Bypass ? "bypass" : g_p.modo == Modo::Fir ? "fir" : "crudo",
                nombre_perfil(g_fir->perfil()), g_hpf.fc(),
                g_p.fuente == FuenteDiag::Local ? "local" : "ext", g_ref_activa ? "on" : "off",
                g_fuente_interna ? "generador" : "uart", g_p.formato == FormatoIn::Adc12 ? "adc12" : "q15", g_p.adc_mid);
}

static void reiniciar_metricas() { g_met.reset(); g_met_fir->reset(); g_tiene_prev = false; }

static const char* nombre_escucha() {
  return g_p.modo == Modo::Fir ? "fir (temporal, Nodo 2)" : g_p.modo == Modo::Crudo ? "raw (entrada sin filtrar)"
       : g_p.modo == Modo::Bypass ? "bypass (FFT+IFFT con G=1)" : "filter (espectral, Nodo 3)";
}

// Las dos rutas lado a lado: la comparacion cuantitativa que pide el enunciado.
static void imprimir_metricas() {
  const Reporte e = g_met.reporte(), f = g_met_fir->reporte();
  Serial.printf("[METODO ACTIVO] GPIO25 (parlante): %s | GPIO26: fir (temporal, Nodo 2)\n", nombre_escucha());
  Serial.printf("                                  ESPECTRAL (Nodo 3)   FIR %s (Nodo 2)\n", nombre_perfil(g_fir->perfil()));
  Serial.printf("[SIN REF] atenuacion en rint      %8.2f dB          %8.2f dB\n", e.atenuacion_rng_db, f.atenuacion_rng_db);
  Serial.printf("          energia util conservada %8.1f %%           %8.1f %%\n", e.util_conservada_pct, f.util_conservada_pct);
  Serial.printf("          SNR est. entrada        %8.2f dB          %8.2f dB   (ESTIMADAS con el piso del Nodo 1,\n", e.snr_est_in_db, f.snr_est_in_db);
  Serial.printf("          SNR est. salida         %8.2f dB          %8.2f dB    no son SNR verdaderos)\n", e.snr_est_out_db, f.snr_est_out_db);
  Serial.printf("          reduccion del piso      %8.2f dB          %8.2f dB   avances=%lu\n", e.reduccion_piso_db, f.reduccion_piso_db,
                (unsigned long)e.hops);
  if (e.ref_muestras) {
    Serial.printf("[CON REF] MSE                     %10.4e        %10.4e   muestras=%lu\n", e.mse, f.mse, (unsigned long)e.ref_muestras);
    Serial.printf("          SNR entrada             %8.2f dB          %8.2f dB\n", e.snr_in, f.snr_in);
    Serial.printf("          SNR salida              %8.2f dB          %8.2f dB\n", e.snr_out, f.snr_out);
    Serial.printf("          mejora dSNR             %8.2f dB          %8.2f dB\n", e.dsnr, f.dsnr);
    Serial.printf("          energia util            %8.1f %%           %8.1f %%\n", e.e_util, f.e_util);
  } else {
    Serial.println("[CON REF] MSE y SNR verdaderos: requieren la senal limpia ('ref on' + referencia por UART1, 'metrics reset').");
  }
  const double avg = g_met.t_blocks ? g_met.t_sum_us / g_met.t_blocks : 0;
  Serial.printf("[TIEMPO] bloques=%lu us/bloque (las dos rutas): prom=%.0f min=%lu max=%lu (presupuesto %lu us) excedidos=%lu\n",
                (unsigned long)g_met.t_blocks, avg, (unsigned long)(g_met.t_blocks ? g_met.t_min_us : 0),
                (unsigned long)g_met.t_max_us, (unsigned long)(1000000ull * N / FS), (unsigned long)g_met.t_excedidos);
}

static void imprimir_estado() {
  const LinkStats& a = g_parser_n1.stats();
  Serial.printf("[ENLACE N1] bytes=%lu pcm_ok=%lu params=%lu piso=%lu crc_err=%lu bytes_basura=%lu\n",
                (unsigned long)a.bytes, (unsigned long)a.pcm_ok, (unsigned long)a.params_ok, (unsigned long)a.piso_ok,
                (unsigned long)a.crc_err, (unsigned long)a.bytes_descartados);
  Serial.printf("[ENTRADA] DC de la ultima trama = %+.4f (escala completa 1.0; un DC alto desplaza la salida del DAC; ajuste 'set adcmid')\n", g_dc_in);
  Serial.printf("[BLOQUES] procesados=%lu perdidos=%lu rellenados=%lu repetidos=%lu resync=%lu desbordes_cola=%lu\n",
                (unsigned long)g_bloques, (unsigned long)g_perdidos, (unsigned long)g_rellenados,
                (unsigned long)g_repetidos, (unsigned long)g_resync, (unsigned long)g_overflow);
  Serial.printf("[DAC] activo=%d escritas=%lu vacios=%lu (muestras %lu) desbordes_anillo=%lu saturadas=%lu errores=%lu anillo=%u/%u (%.0f ms) "
                "reloj=%s ajuste=%+.0f ppm (APLL %.0f Hz, %lu cambios)\n",
                g_audio.stats.activo, (unsigned long)g_audio.stats.escritas, (unsigned long)g_audio.stats.vacios,
                (unsigned long)g_audio.stats.muestras_vacias, (unsigned long)g_audio.stats.desbordes,
                (unsigned long)g_audio.stats.saturadas, (unsigned long)g_audio.stats.errores_dac,
                (unsigned)g_audio.cuadros(), (unsigned)(RING_SAMPLES / CANALES_DAC), 1000.0 * g_audio.cuadros() / FS,
                g_audio.modo_reloj() == ModoReloj::Apll ? "apll" : g_audio.modo_reloj() == ModoReloj::Interp ? "interp" : "off",
                g_audio.ajuste_ppm(), g_audio.apll_nominal_hz(), (unsigned long)g_audio.stats.ajustes_apll);
  Serial.printf("[MEMORIA] heap libre=%lu minimo=%lu total=%lu B; pila loop libre=%u B\n",
                (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getHeapSize(),
                (unsigned)uxTaskGetStackHighWaterMark(nullptr));
  Serial.printf("[LATENCIA] algoritmica=%lu ms (bloque %lu + solape %lu) + DMA %lu ms + anillo %.0f ms; "
                "fallbacks de diagnostico externo=%lu\n",
                (unsigned long)(1000ul * (N + HOP) / FS), (unsigned long)(1000ul * N / FS), (unsigned long)(1000ul * HOP / FS),
                (unsigned long)(1000ul * DAC_DESC_NUM * DAC_DESC_BYTES / 2 / CANALES_DAC / FS), 1000.0 * g_audio.cuadros() / FS,
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

// El ultimo 'scen' se guarda en la flash (NVS): abrir el monitor serie reinicia el ESP32 (DTR/RTS)
// y sin esto el nodo volveria a la banda y al perfil FIR por defecto sin que nadie lo note.
static void guardar_escenario(const char* id) {
  Preferences pr;
  if (pr.begin("nodo3", false)) { pr.putString("scen", id); pr.end(); }
}

static void escenario(const char* id) {
  struct E { const char* id; float blo, bhi, rlo, rhi; PerfilFir fir; };
  static const E t[] = {{"P1", 900, 1100, 2485, 2515, PerfilFir::Tono}, {"P2", 340, 1420, 4500, 7500, PerfilFir::Multitono},
                        {"P3", 150, 2700, 3400, 8000, PerfilFir::Audio}, {"P4", 300, 900, 1600, 8000, PerfilFir::Transitoria},
                        {"P5", 340, 1420, 1000, 2200, PerfilFir::Multitono}};
  for (const E& e : t)
    if (!strcasecmp(id, e.id)) {
      g_p.banda_lo = e.blo; g_p.banda_hi = e.bhi; g_p.rint_lo = e.rlo; g_p.rint_hi = e.rhi;
      g_fir->set_perfil(e.fir); g_stats_fir->reset();
      reiniciar_metricas();
      Serial.printf("Escenario %s: banda util %.0f-%.0f Hz, rango de interferencia %.0f-%.0f Hz, perfil FIR %s. Metricas reiniciadas.\n",
                    e.id, e.blo, e.bhi, e.rlo, e.rhi, nombre_perfil(e.fir));
      guardar_escenario(e.id);
      return;
    }
  Serial.println("P6 (desconocido): fije 'set band lo hi' y 'set rint lo hi' segun la prueba. IDs validos: P1..P5.");
}

// 'recomendar': que metodo conviene para el ruido actual y por que (recomendacion.h). Usa el
// diagnostico del Nodo 1 si esta fresco; si no, el local (snr = psd/piso en el bin del pico).
static void recomendar_ahora() {
  float f[MAX_FREQ]; int8_t sn[MAX_FREQ]; uint8_t fl[MAX_FREQ]; int n = 0;
  ParamsFrame par{};
  bool ext = false;
  if (g_p.fuente == FuenteDiag::Externo && !g_fuente_interna) {
    portENTER_CRITICAL(&g_mux);
    ext = g_ext_par_ok && g_blk_actual - g_ext_par_blk < 5;
    if (ext) par = g_ext_par;
    portEXIT_CRITICAL(&g_mux);
  }
  if (ext) {
    for (int i = 0; i < par.n; ++i) { f[i] = par.f[i].f_hz; sn[i] = par.f[i].snr_db; fl[i] = par.f[i].flags; }
    n = par.n;
  } else {
    const float* psd = g_dlocal.psd();
    for (int i = 0; i < g_diag.n_interf && n < MAX_FREQ; ++i, ++n) {
      const int k = g_diag.interf[i];
      f[n] = k * BIN_HZ; fl[n] = 0x01;
      const float d = 10.0f * log10f((psd[k] + 1e-30f) / (g_diag.piso[k] + 1e-30f));
      sn[n] = int8_t(constrain(lrintf(d), -128L, 127L));
    }
  }
  constexpr size_t LARGO = 2048;          // en el heap: la DRAM estatica del ESP32 clasico esta llena
  static char* texto = static_cast<char*>(malloc(LARGO));
  if (!texto) { Serial.println("ERROR: sin memoria para 'recomendar'"); return; }
  recomendar(f, sn, fl, n, g_diag.piso, g_diag.piso_valido, g_p.banda_lo, g_p.banda_hi, g_fir->perfil(),
             g_p.g_interf, texto, LARGO);
  Serial.printf("[RECOMENDAR] con el diagnostico %s (no cambia nada; aplique con 'mode ...')\n",
                ext ? "del Nodo 1" : "local");
  Serial.print(texto);
}

static void iniciar_captura(const char* arg) {
  if (g_cap_estado != EstadoCaptura::Libre) { Serial.println("#captura en curso"); return; }
  int bloques = arg ? atoi(arg) : 4;
  if (bloques < 1 || bloques > int(CAPTURA_MAX_BLOQUES)) bloques = 4;
  // El heap libre esta repartido en pedazos: si no hay un bloque contiguo para todo lo pedido,
  // se captura lo que entre (bajando de a un bloque) y se avisa.
  const int pedidos = bloques;
  for (; bloques >= 1; --bloques) {
    g_cap_n = size_t(bloques) * N;
    g_cap = static_cast<uint8_t*>(malloc(captura_tamano(g_cap_n)));
    if (g_cap) break;
  }
  if (!g_cap) { Serial.println("#error sin memoria para la captura"); return; }
  if (bloques < pedidos)
    Serial.printf("#captura: no hay memoria contigua para %d bloques; se capturan %d (bloque libre mayor: %lu B)\n",
                  pedidos, bloques, (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  g_cap_idx = 0;
  Serial.printf("#captura %d bloques (%.0f ms); se envia en binario (tools/graficar_captura.py)\n",
                bloques, 1000.0 * g_cap_n / FS);
  g_cap_estado = EstadoCaptura::Grabando;
}

// Envia la trama de captura de a poco (lo que quepa en el bufer del USB): el proceso no se detiene.
static void enviar_captura() {
  if (g_cap_estado != EstadoCaptura::Enviando) return;
  const int libre = Serial.availableForWrite();
  if (libre <= 0) return;
  size_t w = g_cap_largo - g_cap_enviado;
  if (w > size_t(libre)) w = size_t(libre);
  Serial.write(g_cap + g_cap_enviado, w);
  g_cap_enviado += w;
  if (g_cap_enviado >= g_cap_largo) {
    free(g_cap); g_cap = nullptr;
    g_cap_estado = EstadoCaptura::Libre;
  }
}

static void ayuda() {
  Serial.println(
    "Comandos (terminar con Enter):\n"
    "  status | params | metrics [reset] | bench | stream on|off\n"
    "  set alfa|beta|ginterf|lambda <v> | set notch <bins> | set band <lo> <hi> | set rint <lo> <hi>\n"
    "  mode filter|fir|bypass|raw  que se escucha: filter=espectral (Nodo 3), fir=temporal (Nodo 2), raw=entrada sin filtrar\n"
    "  fir tono|multitono|audio|transitoria   perfil del FIR (scen lo elige solo)\n"
    "  recomendar                 que metodo conviene para el ruido actual y por que\n"
    "  reloj apll|interp|off      seguir el reloj del Nodo 1: ajustando el APLL del DAC (defecto) o remuestreando\n"
    "  captura [bloques]          envia entrada + salidas (1-16 bloques) para graficar/escuchar en la PC\n"
    "  hpf <Hz>                   pasa altos extra a la entrada (defecto 0: ya lo aplica el Nodo 1)\n"
    "  diag local|ext             origen del piso/interferentes (ext = tramas del Nodo 1, con respaldo local)\n"
    "  scen P1..P5                banda util y rango de interferencia del escenario\n"
    "  ref on|off                 referencia limpia por UART1 (metricas verdaderas); 'metrics reset' antes de la prueba\n"
 "  fmt adc12|q15 | set adcmid <cuentas>   formato de la trama PCM de entrada (adc12 = cuenta cruda del Nodo 1, centro 2048)\n"
    "  baud <n>                   velocidad de la UART2 (debe coincidir con el Nodo 1; defecto 921600)\n"
    "  src uart|gen               fuente: UART2 (Nodo 1) o generador interno\n"
    "  gen f1 a1 f2 a2 ruido      generador: dos tonos (Hz, amplitud 0-1) + ruido blanco (amplitud)\n"
    "  gain <codigos>             ganancia del DAC (127 = escala completa), dac stop|start");
}

static void comando(char* l) {
  char* a = strtok(l, " \t");
  if (!a) return;
  char* b = strtok(nullptr, " \t"); char* c = strtok(nullptr, " \t"); char* d = strtok(nullptr, " \t");
  char* e = strtok(nullptr, " \t"); char* f = strtok(nullptr, " \t");
  if (!strcmp(a, "help") || !strcmp(a, "?")) ayuda();
  else if (!strcmp(a, "status")) imprimir_estado();
  else if (!strcmp(a, "params")) imprimir_parametros();
  else if (!strcmp(a, "metrics")) { if (b && !strcmp(b, "reset")) { reiniciar_metricas(); Serial.println("metricas reiniciadas"); } else imprimir_metricas(); }
  else if (!strcmp(a, "bench")) benchmark();
  else if (!strcmp(a, "recomendar")) recomendar_ahora();
  else if (!strcmp(a, "captura")) iniciar_captura(b);
  else if (!strcmp(a, "hpf") && b) {
    const float f = constrain(atof(b), 0.0f, 400.0f);
    g_hpf.configurar(f);
    Serial.printf("pasa altos de entrada: %s\n", f > 0 ? String(String(f, 0) + " Hz").c_str() : "apagado");
  }
  else if (!strcmp(a, "reloj") && b) {
    g_audio.set_modo_reloj(!strcmp(b, "interp") ? ModoReloj::Interp : !strcmp(b, "off") ? ModoReloj::Off : ModoReloj::Apll);
    Serial.printf("reloj: %s\n", g_audio.modo_reloj() == ModoReloj::Apll ? "apll (ajusta el reloj del DAC)"
                  : g_audio.modo_reloj() == ModoReloj::Interp ? "interp (remuestrea la salida)" : "off (sin compensar)");
  }
  else if (!strcmp(a, "stream")) {
    g_stream = b && !strcmp(b, "on");
    if (g_stream) Serial.println("#N3,ms,bloques,perdidos,crc,desb_cola,vacios_dac,saturadas,us_prom,us_max,aten_db,util_pct,snr_est_in,snr_est_out,anillo,mse,snr_in,snr_out,dsnr,e_util,aten_fir,util_fir,snr_est_out_fir,mse_fir,dsnr_fir,escucha");
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
    // Solo elige que se escucha por GPIO25: las dos rutas siguen procesando y midiendo.
    // 'bypass' es la excepcion: la ruta espectral deja G = 1 (comprueba la reconstruccion).
    g_p.modo = !strcmp(b, "bypass") ? Modo::Bypass : !strcmp(b, "raw") ? Modo::Crudo : !strcmp(b, "fir") ? Modo::Fir : Modo::Filtro;
    Serial.printf("GPIO25 (parlante): %s\n", nombre_escucha());
    imprimir_parametros();
  }
  else if (!strcmp(a, "fir") && b) {
    const PerfilFir p = !strcmp(b, "tono") ? PerfilFir::Tono : !strcmp(b, "audio") ? PerfilFir::Audio
                      : !strcmp(b, "transitoria") ? PerfilFir::Transitoria : PerfilFir::Multitono;
    g_fir->set_perfil(p); g_stats_fir->reset(); g_met_fir->reset();
    imprimir_parametros();
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
    reiniciar_metricas(); imprimir_parametros(); }
  else if (!strcmp(a, "src") && b) {
    g_fuente_interna = !strcmp(b, "gen"); g_gen_proximo_us = micros();
    int idx;                                     // descartar bloques viejos de la otra fuente
    while (xQueueReceive(g_llenos, &idx, 0) == pdTRUE) xQueueSend(g_libres, &idx, 0);
    resincronizar();                             // las dos fuentes numeran distinto
    imprimir_parametros();
  }
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
  if (!g_stream || millis() < g_prox_estado_ms || g_cap_estado == EstadoCaptura::Enviando) return;
  g_prox_estado_ms = millis() + 1000;
  const Reporte r = g_met.reporte(), f = g_met_fir->reporte();
  const double avg = g_met.t_blocks ? g_met.t_sum_us / g_met.t_blocks : 0;
  // escucha: 0 filter, 1 bypass, 2 raw, 3 fir (lo que sale por GPIO25)
  Serial.printf("N3,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.0f,%lu,%.2f,%.1f,%.2f,%.2f,%u,%.4e,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.2f,%.4e,%.2f,%u\n",
                (unsigned long)millis(), (unsigned long)g_bloques, (unsigned long)g_perdidos,
                (unsigned long)g_parser_n1.stats().crc_err, (unsigned long)g_overflow,
                (unsigned long)g_audio.stats.vacios, (unsigned long)g_audio.stats.saturadas, avg,
                (unsigned long)g_met.t_max_us, r.atenuacion_rng_db, r.util_conservada_pct, r.snr_est_in_db,
                r.snr_est_out_db, (unsigned)g_audio.cuadros(), r.mse, r.snr_in, r.snr_out, r.dsnr, r.e_util,
                f.atenuacion_rng_db, f.util_conservada_pct, f.snr_est_out_db, f.mse, f.dsnr, (unsigned)g_p.modo);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nNodo 3 - filtrado espectral FFT/IFFT (CE1110)");
  g_fir = new (std::nothrow) SalidaFir();
  g_stats_fir = new (std::nothrow) StatsSenales();
  g_met_fir = new (std::nothrow) Metrics();
  g_hpf.configurar(HPF_DEFECTO_HZ);
  if (!g_fir || !g_stats_fir || !g_met_fir) { Serial.println("ERROR: sin memoria para la salida FIR"); for (;;) delay(1000); }
  if (!g_nodo.begin() || !g_dlocal.begin() || !g_stats_fir->begin()) { Serial.println("ERROR: no se pudo iniciar la FFT"); for (;;) delay(1000); }
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
  {                                                  // restaurar el ultimo escenario elegido
    Preferences pr;
    String id;
    if (pr.begin("nodo3", true)) { id = pr.getString("scen", ""); pr.end(); }
    if (id.length()) { Serial.print("(restaurado de la flash) "); escenario(id.c_str()); }
  }
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
  enviar_captura();
  emitir_estado_periodico();
}
