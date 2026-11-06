// Nodo 3 (filtrado espectral FFT/IFFT) - parametros del firmware.
// Los valores de referencia salen de full_system/sistema_ruido/config.py.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace n3 {

// ---- Parametros del sistema (identicos al prototipo Python) ---------------
constexpr uint32_t FS = 16000;           // Hz
constexpr int N = 1024;                  // tamano de FFT = muestras por bloque del enlace
constexpr int HOP = N / 2;               // avance del nodo espectral (50 % de solape)
constexpr int NB = N / 2 + 1;            // bins de la FFT real: 513
constexpr float BIN_HZ = float(FS) / N;  // 15.625 Hz

// Parametros del filtro (modificables en ejecucion con "set ...").
constexpr float ALFA = 3.0f;             // sobre-estimacion del ruido
constexpr float BETA = 0.1f;             // ganancia minima (-20 dB)
constexpr float G_INTERF = 0.01f;        // ganancia en bins interferentes (-40 dB)
constexpr int NOTCH_BINS = 4;            // semiancho del rechazo por interferente

// Estimador local de diagnostico (replica de NodoAdquisicion, Nodo 1).
constexpr float LAMBDA_EMA = 0.9f;
constexpr int K_MEDIANA = 51;
constexpr float UMBRAL_PICO_DB = 15.0f;
constexpr float RANGO_PICO_DB = 40.0f;
constexpr int GUARDA_BINS = 8;
constexpr float F_RED = 60.0f;
constexpr int MAX_INTERF = 64;           // tope de bins interferentes por bloque

// Banda util y rango de interferencia por defecto (escenario P1).
constexpr float BANDA_LO = 900.0f, BANDA_HI = 1100.0f;
constexpr float RINT_LO = 2485.0f, RINT_HI = 2515.0f;

// ---- Enlace UART ----------------------------------------------------------
constexpr uint8_t SYNC0 = 0xA5;
// El segundo byte de sincronia identifica la trama, como en nodo1_esp32s3/trama.h:
constexpr uint8_t SYNC1_PCM = 0x5A;      // muestras (link.py / trama::empaquetar)
constexpr uint8_t SYNC1_DIAG = 0x5B;     // diagnostico del Nodo 1 (trama::empaquetarDiagnostico)
constexpr uint8_t SYNC1_PISO = 0x5D;     // EXTENSION propuesta: piso de ruido submuestreado (0x5C = espectro del Nodo 1, se ignora)
constexpr int MAX_FREQ = 8;              // frecuencias por trama de diagnostico (MAX_REPORTADAS del Nodo 1)
constexpr int PISO_DECIM_MAX = 64;
constexpr int PISO_MAX_KNOTS = 129;      // 513 bins / decimacion 4 como minimo
constexpr size_t MAX_FRAME = 2 + 2 + 2 + 2 * N + 2;  // trama PCM completa = 2056 bytes
constexpr uint32_t BAUD = 921600;        // el mismo BAUD que el firmware del Nodo 1; 460 800 tambien alcanza (69.7 %)
constexpr int POOL_BLOCKS = 4;           // bloques PCM en cola entre recepcion y proceso
constexpr int MAX_FILL_BLOCKS = 4;       // bloques perdidos que se rellenan con ceros

// Pines (ESP32 DevKit 30 pines, ESP-WROOM-32). GPIO16/17 estan libres en WROOM-32,
// pero NO en modulos WROVER/PSRAM. Verificar el modulo real antes de cablear.
constexpr int PIN_RX_NODO1 = 16;         // UART2 RX  <- TX del Nodo 1
constexpr int PIN_TX_NODO1 = 17;         // UART2 TX  -> RX del Nodo 1 (opcional)
constexpr int PIN_RX_REF = 18;           // UART1 RX  <- referencia limpia (solo pruebas)
constexpr int PIN_TX_REF = 19;           // UART1 TX  (sin uso)

// ---- Salida DAC -----------------------------------------------------------
constexpr int DAC_CENTRO = 128;          // codigo de reposo (0 V diferencial = 1.65 V aprox.)
constexpr float DAC_GANANCIA = 60.0f;    // codigo = 128 + 60 x, la misma escala que el Nodo 2 (conversion_dac.h)
constexpr size_t RING_SAMPLES = 4096;    // anillo de salida (potencia de 2): 256 ms
constexpr size_t RING_PREFILL = 1024;    // muestras acumuladas antes de arrancar el DAC
constexpr size_t DAC_CHUNK = 256;        // muestras por escritura al DMA
constexpr int DAC_DESC_NUM = 4;          // descriptores DMA
constexpr int DAC_DESC_BYTES = 512;      // 2 bytes por muestra de 8 bits => 256 muestras
// Latencia del DMA = DAC_DESC_NUM * DAC_DESC_BYTES / 2 / FS = 64 ms

// ---- Metricas -------------------------------------------------------------
constexpr int TEST_BLOQUES = 64;         // duracion de las pruebas del proyecto
constexpr int TEST_MUESTRAS = TEST_BLOQUES * N;

// Modos de operacion.
enum class Modo : uint8_t { Filtro = 0, Bypass = 1, Crudo = 2 };   // Crudo: entrada retardada HOP, sin FFT
enum class FuenteDiag : uint8_t { Local = 0, Externo = 1 };
// Formato de las muestras int16 de la trama PCM de entrada:
//  Adc12: cuenta cruda del ADC del Nodo 1 (0..4095, sin signo); x = (m - adc_mid) / 2048   [firmware actual del Nodo 1]
//  Q15  : entero con signo centrado en 0; x = m / 32768                                  [link.py / simulacion]
enum class FormatoIn : uint8_t { Adc12 = 0, Q15 = 1 };
inline float muestra_a_float(int16_t m, FormatoIn f, float adc_mid) {
  return f == FormatoIn::Adc12 ? (float(m) - adc_mid) * (1.0f / 2048.0f) : float(m) * (1.0f / 32768.0f);
}

// Parametros ajustables en ejecucion.
struct Params {
  float alfa = ALFA;
  float beta = BETA;
  float g_interf = G_INTERF;
  int notch_bins = NOTCH_BINS;
  float banda_lo = BANDA_LO, banda_hi = BANDA_HI;   // banda util [Hz]
  float rint_lo = RINT_LO, rint_hi = RINT_HI;       // rango de interferencia a medir [Hz]
  float lambda_ema = LAMBDA_EMA;
  Modo modo = Modo::Filtro;
  FuenteDiag fuente = FuenteDiag::Local;
  FormatoIn formato = FormatoIn::Adc12;
  float adc_mid = 2048.0f;
};

}  // namespace n3
