// Arnes de PC para el enlace nodo 1 -> nodo 3 (lo usa probar_enlace.py).
// Lee codigos del ADC (int16 little-endian), repite la logica de nodo1_esp32s3.ino
// (trama PCM cada 1024 muestras; analisis cada 512; en los analisis pares, PARAMS y
// PISO detras de su PCM) y escribe el flujo de bytes que saldria por la UART1.
// Uso: enlace_host codigos.bin flujo.bin [banda_lo banda_hi]
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cstring>
#include "../receptor.h"
#include "../trama.h"
#include "../piso_nodo3.h"

int main(int argc, char** argv) {
    if (argc < 3) return 1;
    FILE* f = std::fopen(argv[1], "rb");
    if (!f) return 1;
    std::vector<int16_t> codigos;
    int16_t c;
    while (std::fread(&c, 2, 1, f) == 1) codigos.push_back(c);
    std::fclose(f);
    FILE* out = std::fopen(argv[2], "wb");

    constexpr size_t N = nodo1::N, SALTO = nodo1::SALTO;
    static nodo1::Receptor receptor;
    if (argc >= 5) {                    // banda util opcional (comando b<lo>,<hi> del nodo 1)
        receptor.banda_lo = static_cast<float>(std::atof(argv[3]));
        receptor.banda_hi = static_cast<float>(std::atof(argv[4]));
    }
    static nodo1::PisoNodo3 piso3;
    static int16_t bloque[N];
    static float ventana[N];
    static uint8_t buf[trama::tamano(N)];
    int16_t nudos[nodo1::PisoNodo3::NUDOS];
    size_t llenas = 0, enVentana = 0;
    uint16_t secuencia = 0;
    uint32_t analisis = 0;

    for (int16_t codigo : codigos) {
        bloque[llenas++] = codigo;
        if (llenas == N) {
            std::fwrite(buf, 1, trama::empaquetar(secuencia++, bloque, N, buf), out);
            llenas = 0;
        }
        ventana[enVentana++] = codigo / 4095.0f;
        if (enVentana == N) {
            receptor.analizar(ventana);
            if (analisis % 2 == 0) {
                piso3.actualizar(receptor.potencia);
                const uint8_t sec = static_cast<uint8_t>((secuencia - 1) & 0xFF);
                std::fwrite(buf, 1, trama::empaquetarParams(sec, receptor.validadas,
                            static_cast<size_t>(receptor.n_validadas), buf), out);
                piso3.nudos(nudos);
                std::fwrite(buf, 1, trama::empaquetarPiso(sec, nodo1::PisoNodo3::DECIM, nudos,
                            nodo1::PisoNodo3::NUDOS, buf), out);
                // Lo que se envio, para compararlo con lo que entiende el nodo 3.
                std::printf("ENVIADO %u %d", sec, receptor.n_validadas);
                for (int i = 0; i < receptor.n_validadas; ++i)
                    std::printf(" %.2f,%u", receptor.validadas[i].f_hz, receptor.validadas[i].flags);
                std::printf("\n");
            }
            ++analisis;
            std::memmove(ventana, ventana + SALTO, (N - SALTO) * sizeof(float));
            enVentana = N - SALTO;
        }
    }
    std::fclose(out);
    return 0;
}
