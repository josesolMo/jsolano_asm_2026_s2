// Trama de captura (comando 'captura'): un tramo de la entrada y de las DOS salidas tal como las
// calculo el micro, mas el diagnostico vigente, para graficar espectros y formas de onda en la PC
// (tools/graficar_captura.py). La PC solo dibuja: no procesa la senal del sistema.
//
// Formato (little-endian), CRC-16/CCITT-FALSE sobre todo lo que sigue al SYNC:
//   [A5 5D] [version=1] [escucha u8: Modo] [perfil FIR u8] [n_interf u8]
//   [bloques u16] [n u16] [fs u16] [banda_lo u16] [banda_hi u16] [rint_lo u16] [rint_hi u16]
//   [interf u16 x MAX_INTERF (validos los primeros n_interf)] [piso i16 x NB, centi-dB]
//   [x i16 x n] [espectral i16 x n] [fir i16 x n]   (Q15; las tres alineadas: x retardada HOP)
//   [CRC u16]
#pragma once
#include <math.h>
#include <string.h>
#include "config.h"
#include "communication.h"
#include "spectral_filter.h"

namespace n3 {

constexpr uint8_t SYNC_CAPTURA = 0x5D;
constexpr size_t CAPTURA_ENCABEZADO = 20;
constexpr size_t CAPTURA_MAX_BLOQUES = 16;     // ~1 s; 16 bloques = ~100 KB de heap mientras dura

inline size_t captura_offset_muestras() { return CAPTURA_ENCABEZADO + 2 * MAX_INTERF + 2 * NB; }
inline size_t captura_tamano(size_t n) { return captura_offset_muestras() + 3 * 2 * n + 2; }

namespace detalle {
inline void escribir16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline int16_t a_q15(float v) {
  if (!(v == v)) return 0;
  float q = v * 32768.0f;
  if (q > 32767.0f) q = 32767.0f;
  if (q < -32768.0f) q = -32768.0f;
  return int16_t(lrintf(q));
}
inline uint16_t hz(float f) { return f < 0 ? 0 : (f > 65535.0f ? 65535 : uint16_t(lrintf(f))); }
}  // namespace detalle

// Escribe 'm' muestras de las tres senales a partir de la posicion 'desde'.
inline void captura_muestras(uint8_t* buf, size_t n, size_t desde, const float* x, const float* e,
                             const float* f, size_t m) {
  uint8_t* px = buf + captura_offset_muestras();
  uint8_t* pe = px + 2 * n;
  uint8_t* pf = pe + 2 * n;
  for (size_t i = 0; i < m; ++i) {
    detalle::escribir16(px + 2 * (desde + i), uint16_t(detalle::a_q15(x[i])));
    detalle::escribir16(pe + 2 * (desde + i), uint16_t(detalle::a_q15(e[i])));
    detalle::escribir16(pf + 2 * (desde + i), uint16_t(detalle::a_q15(f[i])));
  }
}

// Encabezado y diagnostico (al terminar de grabar) y CRC. Devuelve el largo total.
inline size_t captura_cerrar(uint8_t* buf, size_t n, uint8_t escucha, uint8_t perfil, const Diag& d,
                             const Params& p) {
  using detalle::escribir16;
  buf[0] = SYNC0; buf[1] = SYNC_CAPTURA; buf[2] = 1; buf[3] = escucha; buf[4] = perfil;
  buf[5] = d.n_interf;
  escribir16(buf + 6, uint16_t(n / N));
  escribir16(buf + 8, uint16_t(n));
  escribir16(buf + 10, uint16_t(FS));
  escribir16(buf + 12, detalle::hz(p.banda_lo));
  escribir16(buf + 14, detalle::hz(p.banda_hi));
  escribir16(buf + 16, detalle::hz(p.rint_lo));
  escribir16(buf + 18, detalle::hz(p.rint_hi));
  uint8_t* q = buf + CAPTURA_ENCABEZADO;
  for (int i = 0; i < MAX_INTERF; ++i) escribir16(q + 2 * i, i < d.n_interf ? d.interf[i] : 0);
  q += 2 * MAX_INTERF;
  for (int k = 0; k < NB; ++k) {
    float db = d.piso_valido && d.piso[k] > 0 ? 1000.0f * log10f(d.piso[k]) : -30000.0f;
    if (db < -32768.0f) db = -32768.0f;
    if (db > 32767.0f) db = 32767.0f;
    escribir16(q + 2 * k, uint16_t(int16_t(lrintf(db))));
  }
  const size_t t = captura_tamano(n);
  escribir16(buf + t - 2, crc16(buf + 2, t - 4));
  return t;
}

}  // namespace n3
