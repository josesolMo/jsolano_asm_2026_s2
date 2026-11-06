// Comando 'recomendar': sugiere el metodo (FIR temporal o espectral) para el ruido presente y
// explica por que, con el diagnostico del Nodo 1 (interferentes validados y piso de ruido).
// No cambia nada: solo imprime. Sin dependencias de Arduino (se prueba en el PC).
//
// Criterio:
//  1. Un interferente angosto por debajo del borde de rechazo del FIR lo atraviesa (el FIR es un
//     pasa bajas): solo el espectral lo elimina (notch en sus bins).
//  2. Del ruido de fondo (piso), el FIR solo quita la parte por encima de su borde de rechazo.
//     Si mucho ruido cae dentro de la banda util, o el FIR quitaria poco, conviene el espectral.
//  3. Si el ruido es casi solo tonal (interferentes >= 10 dB sobre todo el ruido de fondo), el notch
//     espectral lo quita sin tocar la banda util; el FIR tambien lo atenua (se informa cuanto).
//  4. Si no, el FIR basta: fase lineal, 1.3 ms de retardo y sin "ruido musical".
// En la simulacion (full_system, metricas.csv) estas reglas eligen el metodo de mayor dSNR en P1-P5.
#pragma once
#include <math.h>
#include <array>
#include <stdio.h>
#include "config.h"
#include "salida_fir.h"

namespace n3 {

inline void bordes_fir(PerfilFir p, float& paso, float& rechazo) {
  switch (p) {                       // de los comentarios de coeficientes.h (disenar.py)
    case PerfilFir::Tono: paso = 1100; rechazo = 1800; break;
    case PerfilFir::Multitono: paso = 1420; rechazo = 2120; break;
    case PerfilFir::Audio: paso = 2700; rechazo = 3400; break;
    default: paso = 900; rechazo = 1600; break;
  }
}

// |H(f)| del FIR en dB de atenuacion (positivo = atenua), directo de los coeficientes.
template <size_t T>
inline float atenuacion_db(const std::array<float, T>& h, float f) {
  double re = 0, im = 0;
  for (size_t n = 0; n < T; ++n) {
    re += h[n] * cos(2 * M_PI * f * n / FS);
    im -= h[n] * sin(2 * M_PI * f * n / FS);
  }
  return float(-10.0 * log10(re * re + im * im + 1e-30));
}
inline float atenuacion_fir_db(PerfilFir p, float f) {
  switch (p) {
    case PerfilFir::Tono: return atenuacion_db(temporal::TONO, f);
    case PerfilFir::Multitono: return atenuacion_db(temporal::MULTITONO, f);
    case PerfilFir::Audio: return atenuacion_db(temporal::AUDIO, f);
    default: return atenuacion_db(temporal::TRANSITORIA, f);
  }
}

struct Recomendacion {
  Modo modo = Modo::Filtro;
  float frac_banda = 0, frac_sobre = 0, frac_bajo = 0;   // fraccion de la potencia del piso
  float reduccion_fir_db = 0;                            // lo que el FIR podria quitar del piso
  float tonal_sobre_fondo_db = -99;                      // potencia de interferentes / ruido de fondo
  int interf_que_pasan_fir = 0, interf_total = 0;
};

// f_hz/flags: frecuencias validadas (flags & 0x01 = interferente). piso: PSD por bin (NB).
// Escribe la explicacion en 'texto' (si no es nullptr).
// snr_db: altura de cada pico sobre el piso local (de PARAMS). g_interf: ganancia del notch espectral.
inline Recomendacion recomendar(const float* f_hz, const int8_t* snr_db, const uint8_t* flags, int n,
                                const float* piso, bool piso_valido, float banda_lo, float banda_hi,
                                PerfilFir perfil, float g_interf, char* texto, size_t largo) {
  Recomendacion r;
  float paso, rechazo;
  bordes_fir(perfil, paso, rechazo);
  size_t usado = 0;
  auto escribir = [&](const char* fmt, auto... args) {
    if (!texto || usado >= largo) return;
    const int k = snprintf(texto + usado, largo - usado, fmt, args...);
    if (k > 0) usado += size_t(k);
  };

  escribir("Perfil FIR %s: pasa hasta %.0f Hz, rechaza desde %.0f Hz. Banda util %.0f-%.0f Hz.\n",
           nombre_perfil(perfil), paso, rechazo, banda_lo, banda_hi);
  for (int i = 0; i < n; ++i) {
    if (!(flags[i] & 0x01)) continue;
    ++r.interf_total;
    const bool pasa = f_hz[i] < rechazo;
    if (pasa) ++r.interf_que_pasan_fir;
    escribir("  interferente %.1f Hz: %s (FIR: -%.1f dB; notch espectral: -%.0f dB)\n", f_hz[i],
             pasa ? (f_hz[i] < paso ? "dentro de la banda de paso del FIR -> el FIR NO lo quita"
                                    : "en la transicion del FIR -> lo atenua solo en parte")
                  : "sobre el borde de rechazo -> el FIR lo quita",
             atenuacion_fir_db(perfil, f_hz[i]), -20.0f * log10f(g_interf));
  }
  if (r.interf_total == 0) escribir("%s", "  sin interferentes angostos validados\n");

  if (piso_valido) {
    double total = 0, banda = 0, sobre = 0, bajo = 0;
    for (int k = 1; k < NB; ++k) {                   // sin DC
      const double f = k * double(BIN_HZ), p = piso[k];
      total += p;
      if (f >= banda_lo && f <= banda_hi) banda += p;
      if (f >= rechazo) sobre += p;
      if (f < banda_lo) bajo += p;
    }
    if (total > 0) {
      r.frac_banda = float(banda / total);
      r.frac_sobre = float(sobre / total);
      r.frac_bajo = float(bajo / total);
      r.reduccion_fir_db = r.frac_sobre < 0.9999f ? float(-10.0 * log10(1.0 - r.frac_sobre)) : 40.0f;
    }
    // potencia de los interferentes: pico (piso * 10^(snr/10)) por el ancho de ruido de Hann (1.5 bins)
    double tonal = 0;
    for (int i = 0; i < n; ++i) {
      if (!(flags[i] & 0x01)) continue;
      int k = int(lrintf(f_hz[i] / BIN_HZ));
      k = k < 1 ? 1 : (k > NB - 1 ? NB - 1 : k);
      tonal += 1.5 * piso[k] * pow(10.0, snr_db[i] / 10.0);
    }
    if (total > 0 && tonal > 0) r.tonal_sobre_fondo_db = float(10.0 * log10(tonal / total));
    if (tonal > 0) escribir("  interferentes %.1f dB por encima de todo el ruido de fondo\n", r.tonal_sobre_fondo_db);
    escribir("  ruido de fondo: %.0f %% dentro de la banda util, %.0f %% sobre %.0f Hz, %.0f %% bajo la banda\n",
             100 * r.frac_banda, 100 * r.frac_sobre, rechazo, 100 * r.frac_bajo);
    escribir("  el FIR podria reducir el ruido de fondo ~%.1f dB; dentro de la banda util solo el espectral actua\n",
             r.reduccion_fir_db);
  } else {
    escribir("%s", "  sin piso de ruido todavia (esperar unos bloques)\n");
  }

  const char* razon;
  if (r.interf_que_pasan_fir > 0) {
    r.modo = Modo::Filtro;
    razon = "hay interferentes que el FIR deja pasar; el espectral los suprime con un notch";
  } else if (piso_valido && r.tonal_sobre_fondo_db >= 10.0f) {
    r.modo = Modo::Filtro;
    razon = "el ruido es casi solo tonal: el notch espectral lo quita sin tocar la banda util (el FIR tambien lo atenua, ver arriba)";
  } else if (piso_valido && r.frac_banda > 0.25f) {
    r.modo = Modo::Filtro;
    razon = "mucho ruido cae dentro de la banda util: ningun pasa bajas lo separa, la sustraccion espectral si lo reduce";
  } else if (piso_valido && r.reduccion_fir_db < 3.0f) {
    r.modo = Modo::Filtro;
    razon = "el FIR quitaria menos de 3 dB del ruido; el espectral actua en todo el espectro";
  } else {
    r.modo = Modo::Fir;
    razon = "el ruido esta sobre la banda de paso: el FIR lo quita sin artefactos (fase lineal, ~1.3 ms)";
  }
  escribir("RECOMENDACION: mode %s -- %s\n", r.modo == Modo::Fir ? "fir" : "filter", razon);
  return r;
}

}  // namespace n3
