#include "communication.h"
#include <stdint.h>
#include <string.h>

namespace n3 {

uint16_t crc16(const uint8_t* d, size_t n, uint16_t c) {
  for (size_t i = 0; i < n; ++i) {
    c ^= uint16_t(d[i]) << 8;
    for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
  }
  return c;
}

static inline uint16_t rd16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
static inline void wr16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }

// ---- Parser ---------------------------------------------------------------
// Elimina n bytes del inicio; 'descartar' los cuenta como basura (no como trama entregada).
void FrameParser::drop(size_t n, bool descartar) {
  if (n > len_) n = len_;
  if (descartar) st_.bytes_descartados += uint32_t(n);
  memmove(buf_, buf_ + n, len_ - n);
  len_ -= n;
}

// Longitud total de la trama que empieza en buf_ (ya alineada con A5). El segundo byte la identifica.
size_t FrameParser::expected() const {
  if (len_ < 2) return SIZE_MAX;
  switch (buf_[1]) {
    case SYNC1_PCM:
      if (len_ < 6) return SIZE_MAX;
      return rd16(buf_ + 4) == N ? MAX_FRAME : 0;
    case SYNC1_DIAG:
      if (len_ < 9) return SIZE_MAX;
      return buf_[8] <= MAX_FREQ ? 12 + 7 * size_t(buf_[8]) : 0;
    case SYNC1_PISO:
      if (len_ < 8) return SIZE_MAX;
      return (buf_[6] >= 1 && buf_[6] <= PISO_DECIM_MAX && buf_[7] >= 2 && buf_[7] <= PISO_MAX_KNOTS)
                 ? 10 + 2 * size_t(buf_[7]) : 0;
    default:
      return 0;
  }
}

static inline uint32_t rd32(const uint8_t* p) { return uint32_t(rd16(p)) | (uint32_t(rd16(p + 2)) << 16); }

void FrameParser::step(Sink& sink) {
  for (;;) {
    // 1. alinear con la sincronia A5
    if (len_ >= 1 && buf_[0] != SYNC0) {
      const uint8_t* p = (const uint8_t*)memchr(buf_, SYNC0, len_);
      drop(p ? size_t(p - buf_) : len_);
      continue;
    }
    // 2. longitud segun el tipo
    const size_t total = expected();
    if (total == SIZE_MAX) return;
    if (total == 0) { drop(1); continue; }
    if (len_ < total) return;
    // 3. CRC (desde el byte 2, sin la sincronia) y entrega
    const bool ok = crc16(buf_ + 2, total - 4) == rd16(buf_ + total - 2);
    if (ok) {
      switch (buf_[1]) {
        case SYNC1_PCM:                        // little-endian nativo en ESP32 y x86; buf_ alineado a 4
          ++st_.pcm_ok;
          sink.on_pcm(rd16(buf_ + 2), reinterpret_cast<const int16_t*>(buf_ + 6));
          break;
        case SYNC1_DIAG: {
          DiagFrame f;
          f.analisis = rd32(buf_ + 2); f.t_us = rd16(buf_ + 6); f.n = buf_[8];
          for (int i = 0; i < f.n; ++i) {
            const uint8_t* q = buf_ + 10 + 7 * i;
            f.f[i].f_hz = rd16(q) * 0.25f;
            f.f[i].amplitud = rd16(q + 2) * 1e-4f;
            f.f[i].snr_db = int16_t(rd16(q + 4)) * 0.1f;
            f.f[i].flags = q[6];
          }
          ++st_.diag_ok;
          sink.on_diag(f);
          break;
        }
        default: {                             // SYNC1_PISO
          PisoFrame f;
          f.analisis = rd32(buf_ + 2); f.decim = buf_[6]; f.nk = buf_[7];
          for (int i = 0; i < f.nk; ++i) f.centi_db[i] = int16_t(rd16(buf_ + 8 + 2 * i));
          ++st_.piso_ok;
          sink.on_piso(f);
        }
      }
      drop(total, false);
    } else {                                   // falso sync o trama danada: seguir buscando
      ++st_.crc_err;
      drop(1);
    }
  }
}

void FrameParser::feed(const uint8_t* datos, size_t n, Sink& sink) {
  st_.bytes += uint32_t(n);
  while (n > 0) {
    const size_t k = (n < MAX_FRAME - len_) ? n : MAX_FRAME - len_;
    memcpy(buf_ + len_, datos, k);
    len_ += k; datos += k; n -= k;
    step(sink);
  }
}

// ---- Secuencia ----------------------------------------------------------------
int SeqTracker::on_seq(uint32_t seq, uint32_t mod) {
  if (!tiene_) { tiene_ = true; esperada_ = (seq + 1) % mod; return 0; }
  const uint32_t d = (seq + mod - esperada_) % mod;
  if (d >= mod / 2) { ++repetidas; return -1; }      // anterior a la esperada
  perdidas += d;
  esperada_ = (seq + 1) % mod;
  return int(d);
}

// ---- Constructores de tramas --------------------------------------------------
static inline void wr32(uint8_t* p, uint32_t v) { wr16(p, uint16_t(v)); wr16(p + 2, uint16_t(v >> 16)); }
static inline uint16_t sat16(float v, float lo, float hi) {
  v = v < lo ? lo : (v > hi ? hi : v);
  return uint16_t(int32_t(v + (v >= 0 ? 0.5f : -0.5f)));
}

size_t build_pcm(uint8_t* o, uint16_t seq, const int16_t* m) {
  o[0] = SYNC0; o[1] = SYNC1_PCM;
  wr16(o + 2, seq); wr16(o + 4, N);
  memcpy(o + 6, m, 2 * N);
  wr16(o + 6 + 2 * N, crc16(o + 2, 4 + 2 * N));
  return MAX_FRAME;
}
size_t build_diag(uint8_t* o, uint32_t analisis, uint16_t t_us, const FreqDiag* f, uint8_t n) {
  o[0] = SYNC0; o[1] = SYNC1_DIAG;
  wr32(o + 2, analisis); wr16(o + 6, t_us); o[8] = n; o[9] = 0;
  for (int i = 0; i < n; ++i) {
    uint8_t* q = o + 10 + 7 * i;
    wr16(q, sat16(f[i].f_hz * 4.0f, 0.0f, 65535.0f));
    wr16(q + 2, sat16(f[i].amplitud * 10000.0f, 0.0f, 65535.0f));
    wr16(q + 4, sat16(f[i].snr_db * 10.0f, -32768.0f, 32767.0f));
    q[6] = f[i].flags;
  }
  const size_t t = 12 + 7 * size_t(n);
  wr16(o + t - 2, crc16(o + 2, t - 4));
  return t;
}
size_t build_piso(uint8_t* o, uint32_t analisis, uint8_t decim, const int16_t* c, uint8_t nk) {
  o[0] = SYNC0; o[1] = SYNC1_PISO;
  wr32(o + 2, analisis); o[6] = decim; o[7] = nk;
  for (int i = 0; i < nk; ++i) wr16(o + 8 + 2 * i, uint16_t(c[i]));
  const size_t t = 10 + 2 * size_t(nk);
  wr16(o + t - 2, crc16(o + 2, t - 4));
  return t;
}

}  // namespace n3
