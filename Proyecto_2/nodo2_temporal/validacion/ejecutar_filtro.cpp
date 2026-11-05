#include "../filtro_fir.h"
#include "../coeficientes.h"
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

// El ejecutable de escritorio usa la misma clase que se integrara en la placa.
template <std::size_t T>
int ejecutar(const std::array<float, T>& b, const char* archivoEntrada,
             const char* archivoSalida, std::size_t bloque, bool enSitio) {
    std::ifstream entrada(archivoEntrada, std::ios::binary | std::ios::ate);
    if (!entrada) return 2;
    const auto bytes = entrada.tellg();
    if (bytes < 0 || bytes % sizeof(float) != 0) return 2;
    std::vector<float> x(static_cast<std::size_t>(bytes) / sizeof(float));
    entrada.seekg(0);
    if (!entrada.read(reinterpret_cast<char*>(x.data()), bytes)) return 2;
    std::vector<float> y(x.size());
    temporal::FiltroFIR<T> filtro(b);
    // El reinicio debe borrar incluso un historial previamente excitado.
    float previo = 1.0f;
    filtro.procesar(&previo, &previo, 1);
    filtro.reiniciar();
    filtro.procesar(nullptr, nullptr, 0);
    for (std::size_t i = 0; i < x.size(); i += bloque) {
        const auto cantidad = std::min(bloque, x.size() - i);
        filtro.procesar(x.data() + i, enSitio ? x.data() + i : y.data() + i, cantidad);
    }
    // Los archivos binarios permiten comparar cada muestra sin redondeo de texto.
    std::ofstream salida(archivoSalida, std::ios::binary);
    const auto& resultado = enSitio ? x : y;
    salida.write(reinterpret_cast<const char*>(resultado.data()), resultado.size() * sizeof(float));
    return salida ? 0 : 2;
}

// Los perfiles son explicitos: no se infiere la banda util a partir del ruido.
int main(int argc, char** argv) {
    if (argc != 6) return 2;
    const std::string perfil = argv[1];
    const int bloque = std::stoi(argv[4]);
    if (bloque <= 0) return 2;
    const bool enSitio = std::string(argv[5]) == "1";
    if (perfil == "tono") return ejecutar(temporal::TONO, argv[2], argv[3], bloque, enSitio);
    if (perfil == "multitono") return ejecutar(temporal::MULTITONO, argv[2], argv[3], bloque, enSitio);
    if (perfil == "audio") return ejecutar(temporal::AUDIO, argv[2], argv[3], bloque, enSitio);
    if (perfil == "transitoria") return ejecutar(temporal::TRANSITORIA, argv[2], argv[3], bloque, enSitio);
    return 2;
}
