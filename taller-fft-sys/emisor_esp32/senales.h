#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

// Estos parametros pertenecen solo al DAC del emisor, no a la simulacion.
namespace emisor {
constexpr uint32_t FS = 16000;
constexpr uint32_t MUESTRAS = 65536;
constexpr uint32_t RAMPA = 320;
constexpr uint8_t CENTRO = 128;
constexpr double EXCURSION = 80.0;
constexpr double PI_SENAL = 3.14159265358979323846;
enum class Modo : uint8_t { Seno1000 = 1, Seno2500, Multitono, Transitoria, Melodia };

// Los nombres incluyen los parametros que se necesitan verificar en el receptor.
inline const char* descripcion(Modo modo) {
    switch (modo) {
        case Modo::Seno1000: return "Seno 1000 Hz";
        case Modo::Seno2500: return "Seno 2500 Hz (interferente individual)";
        case Modo::Multitono: return "440/880/1320 Hz; amplitudes 1/.55/.30; fases 0/35/-50 grados";
        case Modo::Transitoria: return "Transitoria 600 Hz; puerta 2 Hz; decaimiento exp(-5*(t%1))";
        case Modo::Melodia: return "220/261.63/329.63/440 Hz; seis armonicos";
    }
    return "Modo invalido";
}

// El indice absoluto evita reiniciar fases en los limites de bloques DMA.
inline double muestra_cruda(Modo modo, uint32_t indice) {
    const double t = static_cast<double>(indice) / FS;
    switch (modo) {
        case Modo::Seno1000: return std::sin(2 * PI_SENAL * 1000 * t);
        case Modo::Seno2500: return std::sin(2 * PI_SENAL * 2500 * t);
        case Modo::Multitono:
            return std::sin(2 * PI_SENAL * 440 * t)
                + .55 * std::sin(2 * PI_SENAL * 880 * t + 35 * PI_SENAL / 180)
                + .30 * std::sin(2 * PI_SENAL * 1320 * t - 50 * PI_SENAL / 180);
        case Modo::Transitoria:
            return std::exp(-5 * std::fmod(t, 1.0))
                * (std::sin(2 * PI_SENAL * 2 * t) > 0)
                * std::sin(2 * PI_SENAL * 600 * t);
        case Modo::Melodia: {
            // Cada nota dura 16384 muestras y reinicia su tiempo local como Python.
            constexpr double notas[] = {220, 261.63, 329.63, 440};
            const uint32_t segmento = MUESTRAS / 4;
            const double tau = static_cast<double>(indice % segmento) / FS;
            const double ataque = tau < .01 ? tau / .01 : 1.0;
            double suma = 0;
            for (int k = 1; k <= 6; ++k) {
                suma += std::sin(2 * PI_SENAL * k * notas[indice / segmento] * tau) / k;
            }
            return ataque * std::exp(-1.5 * tau) * suma;
        }
    }
    return 0;
}

// Se preservan las rampas globales sin^2 de 20 ms del prototipo Python.
inline double muestra(Modo modo, uint32_t indice) {
    if (indice >= MUESTRAS) return 0;
    double valor = muestra_cruda(modo, indice);
    uint32_t borde = indice < MUESTRAS / 2 ? indice : MUESTRAS - 1 - indice;
    if (borde < RAMPA) {
        const double rampa = std::sin(PI_SENAL * .5 * borde / (RAMPA - 1));
        valor *= rampa * rampa;
    }
    return valor;
}

// Un unico pico para la señal completa conserva relaciones y envolventes.
inline double pico_global(Modo modo, void (*ceder)() = nullptr) {
    double pico = 0;
    for (uint32_t i = 0; i < MUESTRAS; ++i) {
        const double valor = std::fabs(muestra(modo, i));
        if (valor > pico) pico = valor;
        // La placa puede ceder CPU sin acoplar esta logica matematica a Arduino.
        if (ceder && i % 1024 == 0) ceder();
    }
    return pico > 0 ? pico : 1;
}

// El contador informa saturacion real a 8 bits; no se oculta con un cast.
inline uint8_t cuantizar(double valor, uint32_t& recortes) {
    long codigo = std::lround(CENTRO + EXCURSION * valor);
    if (codigo < 0) { ++recortes; codigo = 0; }
    if (codigo > 255) { ++recortes; codigo = 255; }
    return static_cast<uint8_t>(codigo);
}

// Esta misma funcion se compila en el ESP32 y en la prueba de escritorio.
inline void generar_bloque(Modo modo, uint32_t inicio, uint8_t* destino,
                           size_t cantidad, double pico, uint32_t& recortes) {
    for (size_t i = 0; i < cantidad; ++i) {
        destino[i] = cuantizar(muestra(modo, inicio + i) / pico, recortes);
    }
}
}  // namespace emisor
