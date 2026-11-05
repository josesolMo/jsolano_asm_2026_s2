// Nodo 1 (paso 1): captura continua del ADC a 16 kHz y envio de bloques a la PC.
//
// Placa: ESP32-S3 (Arduino-ESP32 3.3.12, "ESP32S3 Dev Module").
// Entrada: GPIO4 = ADC1, canal 3, atenuacion de 12 dB (0 a ~3.1 V).
// Salida: tramas binarias por el puerto serial USB con el formato de link.py.
// En la PC: python nodo1_esp32s3/captura_nodo1.py --puerto COMx --modo 3
//
// En este paso todavia no hay FFT en el micro: solo se valida la adquisicion.

#include <Arduino.h>
#include "esp_adc/adc_continuous.h"
#include "trama.h"

// Fijar la placa evita compilar por error para el ESP32 clasico del emisor.
#if !CONFIG_IDF_TARGET_ESP32S3
#error "Seleccione la placa ESP32S3 Dev Module."
#endif

constexpr uint32_t FS = 16000;          // Hz, igual que el emisor y la simulacion
constexpr size_t N = 1024;              // muestras por trama (potencia de dos)
constexpr adc_channel_t CANAL = ADC_CHANNEL_3;   // GPIO4 en el ESP32-S3
constexpr uint32_t BAUD = 921600;       // se ignora si la placa usa USB nativo (CDC)

// Cada resultado del ADC continuo ocupa SOC_ADC_DIGI_RESULT_BYTES (4 en el S3).
constexpr size_t BYTES_RESULTADO = SOC_ADC_DIGI_RESULT_BYTES;
constexpr size_t RESULTADOS_POR_LECTURA = 256;               // 16 ms de senal
constexpr size_t BYTES_LECTURA = RESULTADOS_POR_LECTURA * BYTES_RESULTADO;

adc_continuous_handle_t adc = nullptr;
uint8_t crudo[BYTES_LECTURA];           // lo que entrega el DMA en cada lectura
int16_t bloque[N];                      // muestras de 12 bits (0..4095)
uint8_t salida[trama::tamano(N)];       // trama lista para enviar
size_t llenas = 0;
uint16_t secuencia = 0;

// El driver llama esto en interrupcion cuando su almacen interno se llena porque
// el programa no leyo a tiempo: se pierden muestras y hay que reportarlo.
volatile uint32_t desbordes = 0;
static bool IRAM_ATTR alDesbordar(adc_continuous_handle_t, const adc_continuous_evt_data_t*,
                                  void*) {
    desbordes = desbordes + 1;
    return false;
}

void iniciarAdc() {
    // Almacen de 8 lecturas (128 ms): margen mientras se envia una trama.
    adc_continuous_handle_cfg_t manejador = {};
    manejador.max_store_buf_size = 8 * BYTES_LECTURA;
    manejador.conv_frame_size = BYTES_LECTURA;
    ESP_ERROR_CHECK(adc_continuous_new_handle(&manejador, &adc));

    // Un solo canal: todas las conversiones son de GPIO4, a FS exacta.
    adc_digi_pattern_config_t patron = {};
    patron.atten = ADC_ATTEN_DB_12;
    patron.channel = CANAL;
    patron.unit = ADC_UNIT_1;
    patron.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_continuous_config_t config = {};
    config.pattern_num = 1;
    config.adc_pattern = &patron;
    config.sample_freq_hz = FS;
    config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    ESP_ERROR_CHECK(adc_continuous_config(adc, &config));

    adc_continuous_evt_cbs_t eventos = {};
    eventos.on_pool_ovf = alDesbordar;
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc, &eventos, nullptr));
    ESP_ERROR_CHECK(adc_continuous_start(adc));
}

void setup() {
    // Un bufer de envio grande evita que write() bloquee mientras llega el DMA.
    // Debe fijarse antes de begin().
    Serial.setTxBufferSize(2 * sizeof(salida));
    Serial.begin(BAUD);
    iniciarAdc();
}

// Responde 'e' con el estado en texto; el script de la PC ignora lo que no es trama.
void atenderComandos() {
    while (Serial.available()) {
        if (Serial.read() == 'e') {
            Serial.printf("\n#estado fs=%lu N=%u tramas=%u desbordes=%lu\n",
                          static_cast<unsigned long>(FS), static_cast<unsigned>(N),
                          static_cast<unsigned>(secuencia),
                          static_cast<unsigned long>(desbordes));
        }
    }
}

void loop() {
    atenderComandos();

    uint32_t leidos = 0;
    // Espera hasta 100 ms por una lectura; con 16 kHz llega una cada 16 ms.
    if (adc_continuous_read(adc, crudo, BYTES_LECTURA, &leidos, 100) != ESP_OK) return;

    for (uint32_t i = 0; i + BYTES_RESULTADO <= leidos; i += BYTES_RESULTADO) {
        const adc_digi_output_data_t* r = reinterpret_cast<const adc_digi_output_data_t*>(crudo + i);
        // Se descarta cualquier resultado que no sea del canal configurado.
        if (r->type2.unit != 0 || r->type2.channel != CANAL) continue;
        bloque[llenas++] = static_cast<int16_t>(r->type2.data);
        if (llenas == N) {
            const size_t largo = trama::empaquetar(secuencia++, bloque, N, salida);
            Serial.write(salida, largo);
            llenas = 0;
        }
    }
}
