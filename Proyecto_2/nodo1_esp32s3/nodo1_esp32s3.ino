#pragma once
#include <cstddef>
#include <cstdint>

// Formato de trama del enlace, identico a full_system/sistema_ruido/link.py:
//   [A5 5A][secuencia u16][n u16][n x int16][CRC-16 de secuencia+n+datos]
// Todo en little-endian. Esta cabecera no depende de Arduino para poder
// probarla tambien en la computadora (validacion/probar_trama.py).
namespace trama {
constexpr uint8_t SYNC0 = 0xA5;
constexpr uint8_t SYNC1 = 0x5A;
constexpr size_t ENCABEZADO = 6;   // SYNC(2) + secuencia(2) + n(2)
constexpr size_t CRC_BYTES = 2;

// Bytes que ocupa una trama con n muestras.
constexpr size_t tamano(size_t n) { return ENCABEZADO + 2 * n + CRC_BYTES; }

// CRC-16/CCITT-FALSE: polinomio 0x1021, valor inicial 0xFFFF (igual que link.py).
inline uint16_t crc16(const uint8_t* datos, size_t largo, uint16_t crc = 0xFFFF) {
    for (size_t i = 0; i < largo; ++i) {
        crc ^= static_cast<uint16_t>(datos[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Escribe un entero de 16 bits en little-endian.
inline void poner16(uint8_t* destino, uint16_t valor) {
    destino[0] = static_cast<uint8_t>(valor & 0xFF);
    destino[1] = static_cast<uint8_t>(valor >> 8);
}

// Arma la trama en 'destino' (debe tener tamano(n) bytes) y devuelve su largo.
inline size_t empaquetar(uint16_t secuencia, const int16_t* muestras, size_t n,
                         uint8_t* destino) {
    destino[0] = SYNC0;
    destino[1] = SYNC1;
    poner16(destino + 2, secuencia);
    poner16(destino + 4, static_cast<uint16_t>(n));
    for (size_t i = 0; i < n; ++i) {
        poner16(destino + ENCABEZADO + 2 * i, static_cast<uint16_t>(muestras[i]));
    }
    // El CRC cubre secuencia, n y datos; el SYNC queda fuera, como en link.py.
    const size_t cuerpo = 4 + 2 * n;
    poner16(destino + ENCABEZADO + 2 * n, crc16(destino + 2, cuerpo));
    return tamano(n);
}

inline void poner32(uint8_t* destino, uint32_t valor) {
    poner16(destino, static_cast<uint16_t>(valor & 0xFFFF));
    poner16(destino + 2, static_cast<uint16_t>(valor >> 16));
}

// Cierra una trama cuyo cuerpo ocupa 'cuerpo' bytes despues del SYNC: agrega el CRC.
inline size_t cerrar(uint8_t* destino, uint8_t sync1, size_t cuerpo) {
    destino[0] = SYNC0;
    destino[1] = sync1;
    poner16(destino + 2 + cuerpo, crc16(destino + 2, cuerpo));
    return 2 + cuerpo + CRC_BYTES;
}

// Trama de diagnostico (una por bloque analizado, cada 32 ms):
//   [A5 5B][analisis u32][t_us u16][n u8][0 u8]
//   n x [f*4 u16][amplitud*10000 u16][snr*10 i16][flags u8]   [CRC-16]
// El SYNC distinto (5B) hace que el lector de muestras de link.py la ignore.
constexpr uint8_t SYNC_DIAGNOSTICO = 0x5B;
constexpr size_t BYTES_FRECUENCIA = 7;
constexpr size_t tamanoDiagnostico(size_t n) { return 2 + 8 + BYTES_FRECUENCIA * n + CRC_BYTES; }

inline uint16_t saturar16(float v, float lo, float hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return static_cast<uint16_t>(static_cast<int32_t>(v + (v >= 0 ? 0.5f : -0.5f)));
}

template <typename Frecuencia>
inline size_t empaquetarDiagnostico(uint32_t analisis, uint16_t t_us, const Frecuencia* f,
                                    size_t n, uint8_t* destino) {
    uint8_t* p = destino + 2;
    poner32(p, analisis);
    poner16(p + 4, t_us);
    p[6] = static_cast<uint8_t>(n);
    p[7] = 0;
    p += 8;
    for (size_t i = 0; i < n; ++i) {
        poner16(p, saturar16(f[i].f_hz * 4.0f, 0.0f, 65535.0f));
        poner16(p + 2, saturar16(f[i].amplitud * 10000.0f, 0.0f, 65535.0f));
        poner16(p + 4, saturar16(f[i].snr_db * 10.0f, -32768.0f, 32767.0f));
        p[6] = f[i].flags;
        p += BYTES_FRECUENCIA;
    }
    return cerrar(destino, SYNC_DIAGNOSTICO, 8 + BYTES_FRECUENCIA * n);
}

// Trama de espectro (solo a pedido, para validar la FFT contra numpy):
//   [A5 5C][analisis u32][N u16][N x float32 entrada][(N/2+1) x float32 |X|^2][CRC-16]
constexpr uint8_t SYNC_ESPECTRO = 0x5C;
constexpr size_t tamanoEspectro(size_t n) { return 2 + 6 + 4 * n + 4 * (n / 2 + 1) + CRC_BYTES; }

inline size_t empaquetarEspectro(uint32_t analisis, const float* x, const float* potencia,
                                 size_t n, uint8_t* destino) {
    uint8_t* p = destino + 2;
    poner32(p, analisis);
    poner16(p + 4, static_cast<uint16_t>(n));
    p += 6;
    // Los float32 se copian byte a byte: ESP32 y PC son little-endian e IEEE 754.
    for (size_t i = 0; i < n; ++i, p += 4) {
        const uint8_t* b = reinterpret_cast<const uint8_t*>(&x[i]);
        p[0] = b[0]; p[1] = b[1]; p[2] = b[2]; p[3] = b[3];
    }
    for (size_t k = 0; k < n / 2 + 1; ++k, p += 4) {
        const uint8_t* b = reinterpret_cast<const uint8_t*>(&potencia[k]);
        p[0] = b[0]; p[1] = b[1]; p[2] = b[2]; p[3] = b[3];
    }
    return cerrar(destino, SYNC_ESPECTRO, 6 + 4 * n + 4 * (n / 2 + 1));
}
}  // namespace trama