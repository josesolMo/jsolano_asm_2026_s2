#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Piso de ruido que el nodo 1 envia al nodo 3 (trama PISO). Replica el estimador de
// full_system/sistema_ruido/acquisition.py y DiagLocal del nodo 3, para que el nodo 3
// reciba exactamente lo que calcularia por su cuenta:
//   P[k] = |FFT(Hann * (x - media))|^2 / sum(Hann^2),  x = (codigo - 2048) / 2048
//   psd  = EMA de P con lambda = 0.9 (una vez por bloque PCM de 1024 muestras)
//   piso = mediana deslizante de 51 bins (bordes por reflexion, scipy mode="reflect")
// Se envia submuestreado: un nudo cada DECIM bins, en centesimas de dB.
//
// No repite la FFT: reutiliza Receptor::potencia, que es |X|^2 de la misma ventana Hann
// periodica aplicada a code/4095 sin DC. Solo cambia la escala:
//   (code - 2048)/2048 = (4095/2048) * code/4095 - 1   (la constante se va con la media)
// No depende de Arduino: se prueba en la PC (validacion/probar_enlace.py).
namespace nodo1 {

class PisoNodo3 {
public:
    static constexpr int N = 1024;
    static constexpr int BINS = N / 2 + 1;
    static constexpr int K_MEDIANA = 51;
    static constexpr int DECIM = 8;
    static constexpr int NUDOS = (BINS - 1) / DECIM + 1;   // 65: bins 0, 8, ..., 512
    static constexpr float LAMBDA = 0.9f;

    // potencia: |X[k]|^2 del receptor (entrada code/4095). Llamar una vez por bloque PCM.
    void actualizar(const float* potencia) {
        // sum(Hann periodica^2) = 3N/8 = 384 exacto
        constexpr float ESCALA = (4095.0f / 2048.0f) * (4095.0f / 2048.0f) / (3.0f * N / 8.0f);
        for (int k = 0; k < BINS; ++k) {
            const float p = potencia[k] * ESCALA;
            psd_[k] = primero_ ? p : LAMBDA * psd_[k] + (1.0f - LAMBDA) * p;
        }
        primero_ = false;
    }

    void reiniciar() { primero_ = true; }
    bool listo() const { return !primero_; }

    // Nudos en centi-dB (redondeo como numpy: round(100 * 10 log10(max(piso, 1e-30)))).
    void nudos(int16_t* centi_db) const {
        constexpr int H = K_MEDIANA / 2;
        float v[K_MEDIANA];
        for (int j = 0; j < NUDOS; ++j) {
            const int i = j * DECIM;
            for (int m = 0; m < K_MEDIANA; ++m) {
                int idx = i + m - H;
                if (idx < 0) idx = -idx - 1;
                if (idx >= BINS) idx = 2 * BINS - 1 - idx;
                v[m] = psd_[idx];
            }
            std::nth_element(v, v + H, v + K_MEDIANA);
            const float piso = std::max(v[H], 1e-30f);
            const float cdb = 1000.0f * std::log10(piso);
            const float r = std::nearbyint(cdb);
            centi_db[j] = static_cast<int16_t>(std::max(-32768.0f, std::min(32767.0f, r)));
        }
    }

    const float* psd() const { return psd_; }

private:
    float psd_[BINS] = {};
    bool primero_ = true;
};

}  // namespace nodo1
