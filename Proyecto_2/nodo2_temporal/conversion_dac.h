#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace salida_dac {
constexpr uint8_t CENTRO = 128;
// 60 codigos por unidad: aun con ganancia FIR de pico < 1.87 queda margen.
constexpr float ESCALA = 60.0f;
struct Contadores {
    uint32_t recortes = 0;
    uint32_t noFinitas = 0;
};

// La escala es fija entre bloques para preservar amplitudes y evitar bombeo.
inline void convertir(const float* entrada, uint8_t* salida, std::size_t cantidad,
                      Contadores& contadores) {
    for (std::size_t i = 0; i < cantidad; ++i) {
        if (!std::isfinite(entrada[i])) {
            ++contadores.noFinitas;
            salida[i] = CENTRO;
            continue;
        }
        // Limitar antes de redondear evita overflow incluso con entradas finitas enormes.
        const double codigo = CENTRO + static_cast<double>(ESCALA) * entrada[i];
        if (codigo < 0.0) {
            ++contadores.recortes;
            salida[i] = 0;
        } else if (codigo > 255.0) {
            ++contadores.recortes;
            salida[i] = 255;
        } else {
            salida[i] = static_cast<uint8_t>(std::lround(codigo));
        }
    }
}
}  // namespace salida_dac
