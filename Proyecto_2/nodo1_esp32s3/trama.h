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
}  // namespace trama
