// Metricas del Nodo 3.
//  - Modo experimental (con referencia limpia alineada): MSE, SNR entrada/salida, mejora de SNR y
//    energia util conservada, con las mismas formulas que full_system/sistema_ruido/metrics.py.
//  - Modo normal (sin referencia): SOLO magnitudes que se pueden medir o estimar con el modelo
//    espectral; las "estimadas" no son SNR verdaderos. Nunca se inventan MSE ni SNR sin referencia.
#pragma once
#include "config.h"
#include "spectral_filter.h"

namespace n3 {

struct Reporte {
  // --- con referencia (valido solo si ref_muestras > 0) ---
  uint32_t ref_muestras = 0;
  double mse = 0, snr_in = 0, snr_out = 0, dsnr = 0, e_util = 0;
  // --- sin referencia: mediciones y estimaciones del modelo espectral ---
  double atenuacion_rng_db = 0;      // medida: potencia en rint antes/despues de aplicar G
  double util_conservada_pct = 0;    // medida: potencia en banda util despues/antes de G
  double snr_est_in_db = 0, snr_est_out_db = 0;   // ESTIMADAS (piso del Nodo 1), no verdaderas
  double reduccion_piso_db = 0;      // estimada: potencia del piso antes/despues
  double bins_atenuados_medio = 0;
  uint32_t hops = 0;
};

class Metrics {
 public:
  void reset();
  // Siempre: estadisticas espectrales de cada avance.
  void on_hop(const HopStats& h);
  // Con referencia: muestras alineadas en el tiempo (s limpia, x entrada, y salida), n por llamada.
  void on_samples(const float* s, const float* x, const float* y, int n);
  Reporte reporte() const;

  // Tiempo de proceso por bloque (us) y contadores de integridad.
  void on_block_time(uint32_t us);
  uint32_t t_blocks = 0, t_min_us = 0xFFFFFFFF, t_max_us = 0;
  double t_sum_us = 0;
  uint32_t t_excedidos = 0;          // bloques que tardaron mas que su duracion (64 ms)

 private:
  // ventana de medicion identica a la del prototipo: muestras [N, L-N) tras reset
  uint32_t idx_ = 0;
  uint32_t ref_n_ = 0;
  double ss_ = 0, e_in_ = 0, e_out_ = 0, sy_ = 0;
  double rng_in_ = 0, rng_out_ = 0, util_in_ = 0, util_out_ = 0;
  double s_in_ = 0, t_in_ = 0, s_out_ = 0, t_out_ = 0, piso_in_ = 0, piso_out_ = 0, atenuados_ = 0;
  uint32_t hops_ = 0;
};

}  // namespace n3
