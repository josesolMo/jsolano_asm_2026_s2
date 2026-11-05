#include "../conversion_dac.h"
#include "../filtro_fir.h"
#include "../coeficientes.h"
#include <cassert>
#include <limits>

int main() {
    // Comprobar centro, escala, extremos, saturacion y entradas no finitas.
    const float entrada[] = {0, 1, -1, 10, -10, 2, -2,
                            std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity()};
    const uint8_t esperada[] = {128, 188, 68, 255, 0, 248, 8, 255, 128, 128};
    uint8_t salida[10];
    salida_dac::Contadores contadores;
    salida_dac::convertir(entrada, salida, 10, contadores);
    for (size_t i = 0; i < 10; ++i) assert(salida[i] == esperada[i]);
    assert(contadores.recortes == 3 && contadores.noFinitas == 2);
    salida_dac::convertir(nullptr, nullptr, 0, contadores);
    assert(contadores.recortes == 3);

    // La cota l1 de cada perfil verifica margen para cualquier entrada con pico <= 1.
    for (const auto* perfil : {&temporal::TONO, &temporal::AUDIO}) {
        float suma = 0;
        for (float b : *perfil) suma += std::fabs(b);
        assert(suma * salida_dac::ESCALA < 127);
    }
    for (const auto* perfil : {&temporal::MULTITONO, &temporal::TRANSITORIA}) {
        float suma = 0;
        for (float b : *perfil) suma += std::fabs(b);
        assert(suma * salida_dac::ESCALA < 127);
    }
    // Mantener contadores entre bloques permite detectar recortes de toda la captura.
    const float excesiva[] = {3};
    salida_dac::convertir(excesiva, salida, 1, contadores);
    assert(contadores.recortes == 4);
}
