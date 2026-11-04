#pragma once
// Receptor del nodo 1 en C++: FFT, piso de ruido y criterios C1-C6.
//
// Es la traduccion de receptor_fft/src/receptor_fft.py. No depende de Arduino,
// asi que se compila tambien en la PC (validacion/probar_receptor.py) y se
// compara bloque a bloque contra la version de Python.
//
// Uso por bloque de N muestras (50 % de solapamiento, uno cada 32 ms):
//   receptor.analizar(muestras);   // muestras en [0, 1] (codigo ADC / 4095)
//   receptor.validadas[0..n_validadas)  -> frecuencias aceptadas y clasificadas

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace nodo1 {

constexpr int N = 1024;               // muestras por bloque (2^10)
constexpr int BINS = N / 2 + 1;       // k = 0..N/2 de la FFT real
constexpr int SALTO = N / 2;          // 50 % de solapamiento
constexpr float FS = 16000.0f;
constexpr float DF = FS / N;          // 15.625 Hz por bin

// Criterios (mismos valores que receptor_fft.py).
constexpr float UMBRAL_DB = 20.0f;    // C1
constexpr int ANCHO_PISO = 31;        // bins de la ventana del piso
constexpr int RANGO_PISO = 7;         // percentil 25 de 31 = octavo menor (indice 7)
constexpr float FACTOR_PISO = 0.28768207f;   // -ln(1 - 0.25)
constexpr int K_MIN = 3;              // C2
constexpr int MARGEN_NYQUIST = 2;     // C2
constexpr int ANCHO_MAX = 3;          // C3
constexpr float TOLERANCIA = 1.5f * DF;      // C4: movimiento permitido entre bloques
constexpr int K_ACEPTAR = 3;          // C4: 3 de los ultimos 4 bloques
constexpr int K_LIBERAR = 1;          // C4: histeresis
constexpr int MAX_CANDIDATOS = 32;
constexpr int MAX_PISTAS = 16;
constexpr int MAX_REPORTADAS = 8;

constexpr uint8_t FLAG_INTERFERENTE = 0x01;
constexpr uint8_t FLAG_ARMONICO = 0x02;
constexpr uint8_t FLAG_BANDA_UTIL = 0x04;

struct Frecuencia {
    float f_hz;
    float amplitud;      // amplitud del seno en las mismas unidades de la entrada
    float snr_db;        // altura sobre el piso local
    uint8_t flags;
};

struct Pista {
    float f_hz;
    float amplitud;
    float snr_db;
    uint8_t historia;    // 4 bits: bit 0 = bloque mas reciente
    bool validada;
};

class Receptor {
public:
    // Parametros ajustables en ejecucion (sin recompilar).
    float banda_lo = 340.0f;
    float banda_hi = 1420.0f;
    float umbral_db = UMBRAL_DB;

    // Resultados del ultimo bloque.
    float potencia[BINS];        // |X[k]|^2
    float piso[BINS];            // potencia media estimada del ruido por bin
    Frecuencia validadas[MAX_REPORTADAS];
    int n_validadas = 0;
    uint32_t pistas_descartadas = 0;   // pistas que no cupieron (se reporta)

    Receptor() {
        // Tablas precalculadas: evitan senos y cosenos en cada bloque.
        for (int n = 0; n < N; ++n) {
            ventana_[n] = 0.5f - 0.5f * std::cos(2.0f * PI_ * n / N);
        }
        for (int k = 0; k < N / 2; ++k) {
            cos_[k] = std::cos(2.0f * PI_ * k / N);
            sin_[k] = -std::sin(2.0f * PI_ * k / N);   // W = e^{-j 2 pi k / N}
        }
        int bits = 0;
        while ((1 << bits) < N) ++bits;
        for (int n = 0; n < N; ++n) {
            int r = 0;
            for (int b = 0; b < bits; ++b) r |= ((n >> b) & 1) << (bits - 1 - b);
            inverso_[n] = static_cast<uint16_t>(r);
        }
    }

    // Paso 2: quitar DC, ventana Hann y FFT. Deja |X[k]|^2 en potencia[].
    void espectro(const float* x) {
        float media = 0.0f;
        for (int n = 0; n < N; ++n) media += x[n];
        media /= N;
        // Se carga en orden de bits invertidos para hacer la FFT en el mismo arreglo.
        for (int n = 0; n < N; ++n) {
            re_[inverso_[n]] = (x[n] - media) * ventana_[n];
            im_[inverso_[n]] = 0.0f;
        }
        fft();
        for (int k = 0; k < BINS; ++k) potencia[k] = re_[k] * re_[k] + im_[k] * im_[k];
    }

    // Paso 3: todo el receptor sobre un bloque.
    void analizar(const float* x) {
        espectro(x);
        estimarPiso();
        buscarCandidatos();
        seguir();
        clasificar();
    }

private:
    static constexpr float PI_ = 3.14159265358979f;

    float ventana_[N];
    float cos_[N / 2];
    float sin_[N / 2];
    uint16_t inverso_[N];
    float re_[N];
    float im_[N];
    float pdb_[BINS];
    Frecuencia cand_[MAX_CANDIDATOS];
    int n_cand_ = 0;
    Pista pistas_[MAX_PISTAS];
    int n_pistas_ = 0;

    // FFT radix-2 iterativa (Cooley-Tukey) sobre re_/im_ ya en orden invertido.
    void fft() {
        for (int tam = 2; tam <= N; tam <<= 1) {
            const int mitad = tam / 2;
            const int paso = N / tam;
            for (int i = 0; i < N; i += tam) {
                for (int j = 0; j < mitad; ++j) {
                    const float wr = cos_[j * paso];
                    const float wi = sin_[j * paso];
                    const int a = i + j;
                    const int b = a + mitad;
                    // Mariposa: t = W * X[b]; X[b] = X[a] - t; X[a] = X[a] + t.
                    const float tr = wr * re_[b] - wi * im_[b];
                    const float ti = wr * im_[b] + wi * re_[b];
                    re_[b] = re_[a] - tr;
                    im_[b] = im_[a] - ti;
                    re_[a] += tr;
                    im_[a] += ti;
                }
            }
        }
    }

    static float aDb(float p) { return 10.0f * std::log10(std::max(p, 1e-30f)); }

    // Percentil 25 de 31 bins con bordes reflejados (igual que scipy, mode='reflect').
    void estimarPiso() {
        float vecinos[ANCHO_PISO];
        const int r = ANCHO_PISO / 2;
        for (int k = 0; k < BINS; ++k) {
            for (int o = -r; o <= r; ++o) {
                int i = k + o;
                if (i < 0) i = -i - 1;
                else if (i >= BINS) i = 2 * BINS - i - 1;
                vecinos[o + r] = potencia[i];
            }
            std::nth_element(vecinos, vecinos + RANGO_PISO, vecinos + ANCHO_PISO);
            piso[k] = vecinos[RANGO_PISO] / FACTOR_PISO;
        }
        for (int k = 0; k < BINS; ++k) pdb_[k] = aDb(potencia[k]);
    }

    int ancho6db(int k) const {
        const float limite = potencia[k] / 4.0f;
        int izq = k, der = k;
        while (izq > 0 && potencia[izq - 1] >= limite) --izq;
        while (der < BINS - 1 && potencia[der + 1] >= limite) ++der;
        return der - izq + 1;
    }

    // C1, C2 y C3 sobre los maximos locales; interpolacion parabolica.
    void buscarCandidatos() {
        n_cand_ = 0;
        const int k_max = N / 2 - MARGEN_NYQUIST;
        for (int k = 1; k < BINS - 1; ++k) {
            if (!(potencia[k] > potencia[k - 1] && potencia[k] >= potencia[k + 1])) continue;
            const float snr = pdb_[k] - aDb(piso[k]);
            if (snr < umbral_db) continue;                    // C1
            if (k < K_MIN || k > k_max) continue;             // C2
            if (ancho6db(k) > ANCHO_MAX) continue;            // C3
            if (n_cand_ == MAX_CANDIDATOS) break;
            const float a = pdb_[k - 1], b = pdb_[k], c = pdb_[k + 1];
            const float den = a - 2.0f * b + c;
            const float d = (den == 0.0f) ? 0.0f : 0.5f * (a - c) / den;
            const float pico_db = b - 0.25f * (a - c) * d;
            Frecuencia& f = cand_[n_cand_++];
            f.f_hz = (k + d) * DF;
            // |X| = A * suma(w) / 2 y suma(w) = N/2 para Hann.
            f.amplitud = 4.0f * std::pow(10.0f, pico_db / 20.0f) / N;
            f.snr_db = snr;
            f.flags = 0;
        }
    }

    // C4: asocia candidatos a pistas, actualiza la historia y aplica la histeresis.
    void seguir() {
        bool usado[MAX_CANDIDATOS] = {};
        for (int p = 0; p < n_pistas_; ++p) {
            Pista& pista = pistas_[p];
            int mejor = -1;
            float distancia = 0.0f;
            for (int i = 0; i < n_cand_; ++i) {
                if (usado[i]) continue;
                const float d = std::fabs(cand_[i].f_hz - pista.f_hz);
                if (mejor < 0 || d < distancia) { mejor = i; distancia = d; }
            }
            pista.historia = static_cast<uint8_t>((pista.historia << 1) & 0x0F);
            if (mejor >= 0 && distancia <= TOLERANCIA) {
                usado[mejor] = true;
                pista.f_hz = 0.7f * pista.f_hz + 0.3f * cand_[mejor].f_hz;
                pista.amplitud = cand_[mejor].amplitud;
                pista.snr_db = cand_[mejor].snr_db;
                pista.historia |= 1;
            }
        }
        for (int i = 0; i < n_cand_; ++i) {
            if (usado[i]) continue;
            if (n_pistas_ == MAX_PISTAS) { ++pistas_descartadas; continue; }
            pistas_[n_pistas_++] = {cand_[i].f_hz, cand_[i].amplitud, cand_[i].snr_db, 1, false};
        }
        int quedan = 0;
        for (int p = 0; p < n_pistas_; ++p) {
            Pista& pista = pistas_[p];
            const int presentes = __builtin_popcount(pista.historia);
            if (presentes >= K_ACEPTAR) pista.validada = true;
            else if (presentes <= K_LIBERAR) pista.validada = false;
            // Una pista sin apariciones en 4 bloques se elimina; el orden se conserva.
            if (pista.historia != 0) pistas_[quedan++] = pista;
        }
        n_pistas_ = quedan;
    }

    // C5: util / interferente / armonico; ordena y recorta a MAX_REPORTADAS.
    void clasificar() {
        Frecuencia todas[MAX_PISTAS];
        int n = 0;
        for (int p = 0; p < n_pistas_; ++p) {
            if (!pistas_[p].validada) continue;
            const Pista& s = pistas_[p];
            const bool util = s.f_hz >= banda_lo && s.f_hz <= banda_hi;
            todas[n++] = {s.f_hz, s.amplitud, s.snr_db, util ? FLAG_BANDA_UTIL : FLAG_INTERFERENTE};
        }
        // f0 = interferente de menor frecuencia; los demas cerca de h*f0 son armonicos.
        float f0 = 0.0f;
        for (int i = 0; i < n; ++i) {
            if ((todas[i].flags & FLAG_INTERFERENTE) && (f0 == 0.0f || todas[i].f_hz < f0)) {
                f0 = todas[i].f_hz;
            }
        }
        if (f0 > 0.0f) {
            for (int i = 0; i < n; ++i) {
                if (!(todas[i].flags & FLAG_INTERFERENTE) || todas[i].f_hz == f0) continue;
                const float h = std::nearbyint(todas[i].f_hz / f0);   // redondeo al par, como Python
                if (h >= 2.0f && std::fabs(todas[i].f_hz - h * f0) <= DF) {
                    todas[i].flags |= FLAG_ARMONICO;
                }
            }
        }
        // Primero interferentes; dentro de cada grupo, de mayor a menor amplitud.
        std::stable_sort(todas, todas + n, [](const Frecuencia& a, const Frecuencia& b) {
            const bool ia = a.flags & FLAG_INTERFERENTE, ib = b.flags & FLAG_INTERFERENTE;
            if (ia != ib) return ia;
            return a.amplitud > b.amplitud;
        });
        n_validadas = std::min(n, MAX_REPORTADAS);
        for (int i = 0; i < n_validadas; ++i) validadas[i] = todas[i];
    }
};

}  // namespace nodo1