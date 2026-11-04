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

// Longitud total de la trama que empieza en buf_, deducida de los primeros 6 bytes.
size_t FrameParser::expected() const {
  const uint8_t tipo = buf_[2];
  const uint8_t b4 = buf_[4], b5 = buf_[5];
  if (b4 == 0 && b5 == 0x04) {                 // n = 1024 en little-endian => PCM
    if (tipo == TIPO_PARAMS) {                 // PARAMS vacio (n=0) con crc_lo = 0x04: desambiguar por CRC
      if (len_ < 7) return SIZE_MAX;
      if (crc16(buf_, 5) == rd16(buf_ + 5)) return 7;
    }
    return MAX_FRAME;
  }
  if (tipo == TIPO_PARAMS && b4 <= MAX_FREQ) return 7 + 4 * size_t(b4);
  if (tipo == TIPO_PISO && b4 >= 1 && b4 <= PISO_DECIM_MAX && b5 >= 2 && b5 <= PISO_MAX_KNOTS)
    return 8 + 2 * size_t(b5);
  return 0;
}

void FrameParser::step(Sink& sink) {
  for (;;) {
    // 1. alinear con SYNC
    if (len_ >= 1 && buf_[0] != SYNC0) {
      const uint8_t* p = (const uint8_t*)memchr(buf_, SYNC0, len_);
      drop(p ? size_t(p - buf_) : len_);
      continue;
    }
    if (len_ >= 2 && buf_[1] != SYNC1) { drop(1); continue; }
    if (len_ < 6) return;
    // 2. longitud
    const size_t total = expected();
    if (total == SIZE_MAX) return;
    if (total == 0) { drop(1); continue; }
    if (len_ < total) return;
    // 3. CRC y entrega
    bool ok = false;
    if (total == MAX_FRAME) {                  // PCM: el CRC cubre seq+n+datos
      ok = crc16(buf_ + 2, total - 4) == rd16(buf_ + total - 2);
      if (ok) {
        int16_t m[N];
        memcpy(m, buf_ + 6, sizeof m);        // little-endian nativo en ESP32 y x86
        ++st_.pcm_ok;
        sink.on_pcm(rd16(buf_ + 2), m);
      }
    } else if (buf_[2] == TIPO_PARAMS) {
      ok = crc16(buf_, total - 2) == rd16(buf_ + total - 2);
      if (ok) {
        ParamsFrame f; f.sec = buf_[3]; f.n = buf_[4];
        for (int i = 0; i < f.n; ++i) {
          const uint8_t* q = buf_ + 5 + 4 * i;
          f.f[i].f_hz = rd16(q) * 0.25f;
          f.f[i].snr_db = int8_t(q[2]);
          f.f[i].flags = q[3];
        }
        ++st_.params_ok;
        sink.on_params(f);
      }
    } else {                                   // PISO
      ok = crc16(buf_, total - 2) == rd16(buf_ + total - 2);
      if (ok) {
        PisoFrame f; f.sec = buf_[3]; f.decim = buf_[4]; f.nk = buf_[5];
        for (int i = 0; i < f.nk; ++i) f.centi_db[i] = int16_t(rd16(buf_ + 6 + 2 * i));
        ++st_.piso_ok;
        sink.on_piso(f);
      }
    }
    if (ok) drop(total, false);
    else { ++st_.crc_err; drop(1); }           // falso sync o trama danada: seguir buscando
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
size_t build_pcm(uint8_t* o, uint16_t seq, const int16_t* m) {
  o[0] = SYNC0; o[1] = SYNC1;
  wr16(o + 2, seq); wr16(o + 4, N);
  memcpy(o + 6, m, 2 * N);
  wr16(o + 6 + 2 * N, crc16(o + 2, 4 + 2 * N));
  return MAX_FRAME;
}
size_t build_params(uint8_t* o, uint8_t sec, const FreqParam* f, uint8_t n) {
  o[0] = SYNC0; o[1] = SYNC1; o[2] = TIPO_PARAMS; o[3] = sec; o[4] = n;
  for (int i = 0; i < n; ++i) {
    wr16(o + 5 + 4 * i, uint16_t(f[i].f_hz * 4.0f + 0.5f));
    o[7 + 4 * i] = uint8_t(f[i].snr_db); o[8 + 4 * i] = f[i].flags;
  }
  const size_t t = 7 + 4 * size_t(n);
  wr16(o + t - 2, crc16(o, t - 2));
  return t;
}
size_t build_piso(uint8_t* o, uint8_t sec, uint8_t decim, const int16_t* c, uint8_t nk) {
  o[0] = SYNC0; o[1] = SYNC1; o[2] = TIPO_PISO; o[3] = sec; o[4] = decim; o[5] = nk;
  for (int i = 0; i < nk; ++i) wr16(o + 6 + 2 * i, uint16_t(c[i]));
  const size_t t = 8 + 2 * size_t(nk);
  wr16(o + t - 2, crc16(o, t - 2));
  return t;
}

}  // namespace n3
