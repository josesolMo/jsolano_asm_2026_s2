#include "../senales.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    // Las exportaciones temporales provienen de la misma cabecera que usa la placa.
    if (argc != 2) return 1;
    for (int numero = 1; numero <= 5; ++numero) {
        const auto modo = static_cast<emisor::Modo>(numero);
        const double pico = emisor::pico_global(modo);
        std::vector<uint8_t> completa(emisor::MUESTRAS + 4096);
        std::vector<uint8_t> bloques(completa.size());
        uint32_t recortes = 0;
        emisor::generar_bloque(modo, 0, completa.data(), completa.size(), pico, recortes);
        // Particiones distintas al DMA revelan reinicios de fase o escala por bloque.
        for (size_t i = 0; i < bloques.size(); i += 997) {
            emisor::generar_bloque(modo, i, bloques.data() + i,
                                  std::min(size_t(997), bloques.size() - i), pico, recortes);
        }
        if (completa != bloques || recortes != 0) return 2;
        for (size_t i = emisor::MUESTRAS; i < completa.size(); ++i) {
            if (completa[i] != emisor::CENTRO) return 3;
        }
        const std::string ruta = std::string(argv[1]) + "/modo_" + std::to_string(numero) + ".bin";
        std::ofstream salida(ruta, std::ios::binary);
        salida.write(reinterpret_cast<const char*>(completa.data()), completa.size());
        if (!salida) return 4;
        std::cout << "Modo " << numero << ": pico=" << pico << "; recortes=" << recortes
                  << "; particiones identicas; cola central correcta\n";
    }
    // La saturacion defensiva se prueba tambien con valores deliberadamente invalidos.
    uint32_t recortes = 0;
    if (emisor::cuantizar(-10, recortes) != 0 || emisor::cuantizar(10, recortes) != 255
        || recortes != 2) return 5;
    return 0;
}
