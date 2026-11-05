// Enlace entre nodos: CRC-16, parser de tramas y recepcion de bytes.
//
// Las tramas son las del Nodo 1 (Proyecto_2/nodo1_esp32s3/trama.h), little-endian, CRC-16/CCITT-FALSE
// calculado desde el byte 2 (sin los dos de sincronia) hasta el final de los datos:
//  PCM   [A5 5A][seq u16][n u16 = 1024][n x int16][CRC]                              link.py / trama::empaquetar
//  DIAG  [A5 5B][analisis u32][t_us u16][n u8][0][n x (f*4 u16, amp*1e4 u16, snr*10 i16, flags u8)][CRC]
//  PISO  [A5 5D][analisis u32][decim u8][nk u8][nk x int16 centi-dB][CRC]            EXTENSION propuesta
// Una trama [A5 5C] (espectro, solo a peticion del Nodo 1) no se usa: se descarta byte a byte.
// El medio fisico (UART hoy; SPI previsto por el Nodo 1) no cambia el formato.
#pragma once
#include "config.h"

namespace n3 {

// CRC-16/CCITT-FALSE (poli 0x1021, init 0xFFFF). crc16("123456789") == 0x29B1.
uint16_t crc16(const uint8_t* datos, size_t n, uint16_t crc = 0xFFFF);

struct FreqDiag {
  float f_hz;
  float amplitud;
  float snr_db;
  uint8_t flags;      // 0x01 interferente, 0x02 armonico, 0x04 banda util
};
struct DiagFrame {
  uint32_t analisis;  // contador de analisis del Nodo 1 (uno cada 32 ms)
  uint16_t t_us;
  uint8_t n;
  FreqDiag f[MAX_FREQ];
};
struct PisoFrame {
  uint32_t analisis;
  uint8_t decim;
  uint8_t nk;
  int16_t centi_db[PISO_MAX_KNOTS];
};

struct LinkStats {
  uint32_t bytes = 0;
  uint32_t pcm_ok = 0, diag_ok = 0, piso_ok = 0;
  uint32_t crc_err = 0;           // tramas completas con CRC incorrecto
  uint32_t bytes_descartados = 0; // bytes ignorados mientras se busca SYNC
};

// Receptor de las tramas de un flujo de bytes arbitrario (fragmentado como sea).
class FrameParser {
 public:
  struct Sink {
    // 'muestras' apunta al buffer interno: copiar antes de volver.
    virtual void on_pcm(uint16_t seq, const int16_t* muestras) = 0;
    virtual void on_diag(const DiagFrame&) {}
    virtual void on_piso(const PisoFrame&) {}
  };
  void feed(const uint8_t* datos, size_t n, Sink& sink);
  void reset() { len_ = 0; }
  const LinkStats& stats() const { return st_; }
  void clear_stats() { st_ = LinkStats(); }
 private:
  void step(Sink& sink);
  void drop(size_t n, bool descartar = true);
  size_t expected() const;        // 0 = invalida, SIZE_MAX = faltan bytes para saberlo
  alignas(4) uint8_t buf_[MAX_FRAME];
  size_t len_ = 0;
  LinkStats st_;
};

// Seguimiento de secuencia con envoltura (u16 para PCM, u8 para parametros).
class SeqTracker {
 public:
  // Devuelve el numero de tramas perdidas antes de esta (>= 0), o -1 si es repetida/antigua.
  int on_seq(uint32_t seq, uint32_t mod);
  void reset() { tiene_ = false; }
  uint32_t perdidas = 0, repetidas = 0;
 private:
  bool tiene_ = false;
  uint32_t esperada_ = 0;
};

// Construccion de tramas (usada por las pruebas y por el modo de autoprueba).
size_t build_pcm(uint8_t* out, uint16_t seq, const int16_t* muestras);
size_t build_diag(uint8_t* out, uint32_t analisis, uint16_t t_us, const FreqDiag* f, uint8_t n);
size_t build_piso(uint8_t* out, uint32_t analisis, uint8_t decim, const int16_t* centi_db, uint8_t nk);

}  // namespace n3
