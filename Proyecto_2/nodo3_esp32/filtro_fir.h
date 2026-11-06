// Copia de nodo2_temporal/filtro_fir.h (Arduino solo compila archivos de la carpeta del sketch).
#pragma once
#include <array>
#include <cstddef>

namespace temporal {
// El tamano fijo evita asignaciones dinamicas durante el procesamiento embebido.
template <std::size_t T>
class FiltroFIR {
    static_assert(T > 0, "El filtro necesita coeficientes");
    std::array<float, T> coeficientes_;
    std::array<float, T> historial_{};
    std::size_t posicion_ = 0;

public:
    // Copiar los coeficientes evita depender de la vida util de un arreglo externo.
    explicit FiltroFIR(const std::array<float, T>& coeficientes)
        : coeficientes_(coeficientes) {}

    // Solo reiniciar al comenzar una captura independiente, nunca entre bloques.
    void reiniciar() {
        historial_.fill(0.0f);
        posicion_ = 0;
    }

    // Entrada centrada en cero; salida sin recortar para conservar la respuesta LTI.
    // Se admite entrada == salida; otros solapamientos de memoria no se admiten.
    void procesar(const float* entrada, float* salida, std::size_t cantidad) {
        for (std::size_t n = 0; n < cantidad; ++n) {
            historial_[posicion_] = entrada[n];
            float acumulado = 0.0f;
            std::size_t indice = posicion_;
            // El recorrido circular conserva exactamente las muestras de bloques previos.
            for (std::size_t k = 0; k < T; ++k) {
                acumulado += coeficientes_[k] * historial_[indice];
                indice = indice == 0 ? T - 1 : indice - 1;
            }
            salida[n] = acumulado;
            posicion_ = (posicion_ + 1) % T;
        }
    }
};
}  // namespace temporal
