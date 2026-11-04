// Nodo 1: adquisicion y analisis en el ESP32-S3.
//
// Placa: ESP32-S3 (Arduino-ESP32 3.3.12, "ESP32S3 Dev Module").
// Entrada: GPIO4 = ADC1, canal 3, atenuacion de 12 dB (0 a ~3.1 V).
//
// Cada 1024 muestras (64 ms) envia una trama con las muestras crudas (formato
// link.py). Cada 512 muestras (32 ms) analiza los ultimos 1024 valores (50 % de
// solapamiento): quita DC, Hann, FFT, piso de ruido y criterios C1-C5
// (receptor.h), y envia una trama de diagnostico con las frecuencias validadas.
//
// Comandos por el monitor serial (una letra):
//   e  estado (texto)     f  enviar el espectro del proximo bloque (validar la FFT)
//   t  encender/apagar un tono de prueba de 1000 Hz (PWM) en GPIO5; con un jumper
//      GPIO5 -> GPIO4 la placa se mide a si misma sin otro equipo
//
// En la PC: python nodo1_esp32s3/validar_nodo1.py --puerto COMx

#include <Arduino.h>
#include <cstring>
#include "esp_adc/adc_continuous.h"
#include "trama.h"
#include "receptor.h"

// Fijar la placa evita compilar por error para el ESP32 clasico del emisor.
#if !CONFIG_IDF_TARGET_ESP32S3
#error "Seleccione la placa ESP32S3 Dev Module."
#endif

constexpr uint32_t FS = 16000;          // Hz, igual que el emisor y la simulacion
constexpr size_t N = nodo1::N;          // 1024 muestras por trama y por analisis
constexpr size_t SALTO = nodo1::SALTO;  // un analisis cada 512 muestras nuevas
constexpr adc_channel_t CANAL = ADC_CHANNEL_3;   // GPIO4 en el ESP32-S3
constexpr uint32_t BAUD = 921600;       // se ignora si la placa usa USB nativo (CDC)
constexpr uint8_t PIN_PRUEBA = 5;       // GPIO5, al lado de GPIO4 en la placa
constexpr uint32_t F_PRUEBA = 1000;     // Hz; el PWM sale del reloj de 80 MHz (preciso)

// Cada resultado del ADC continuo ocupa SOC_ADC_DIGI_RESULT_BYTES (4 en el S3).
constexpr size_t BYTES_RESULTADO = SOC_ADC_DIGI_RESULT_BYTES;
constexpr size_t RESULTADOS_POR_LECTURA = 256;               // 16 ms de senal
constexpr size_t BYTES_LECTURA = RESULTADOS_POR_LECTURA * BYTES_RESULTADO;

adc_continuous_handle_t adc = nullptr;
uint8_t crudo[BYTES_LECTURA];           // lo que entrega el DMA en cada lectura

// Ruta de muestras: trama de 1024 codigos de 12 bits.
int16_t bloque[N];
size_t llenas = 0;
uint16_t secuencia = 0;
uint8_t salidaMuestras[trama::tamano(N)];

// Ruta de analisis: los ultimos N valores en [0, 1] y el receptor.
float ventana[N];
size_t enVentana = 0;                   // cuantas muestras validas hay (hasta N)
uint32_t analisis = 0;
nodo1::Receptor receptor;               // global: ~26 KB, no cabe en la pila de loop()
uint8_t salidaDiagnostico[trama::tamanoDiagnostico(nodo1::MAX_REPORTADAS)];
uint8_t salidaEspectro[trama::tamanoEspectro(N)];
bool pedirEspectro = false;
bool tonoPrueba = false;
uint32_t tMaxUs = 0;                    // peor tiempo de analisis observado

// El driver llama esto en interrupcion cuando su almacen interno se llena porque
// el programa no leyo a tiempo: se pierden muestras y hay que reportarlo.
volatile uint32_t desbordes = 0;
static bool IRAM_ATTR alDesbordar(adc_continuous_handle_t, const adc_continuous_evt_data_t*,
                                  void*) {
    desbordes = desbordes + 1;
    return false;
}

void iniciarAdc() {
    // Almacen de 8 lecturas (128 ms): margen mientras se analiza y se envia.
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
    // El bufer de envio debe fijarse antes de begin(); cabe la trama de espectro.
    Serial.setTxBufferSize(sizeof(salidaEspectro) + sizeof(salidaMuestras));
    Serial.begin(BAUD);
    iniciarAdc();
}

// Los comandos se atienden entre lecturas; el script de la PC ignora el texto.
void atenderComandos() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == 'e') {
            Serial.printf("\n#estado fs=%lu N=%u tramas=%u analisis=%lu desbordes=%lu "
                          "t_max_us=%lu pistas_descartadas=%lu\n",
                          static_cast<unsigned long>(FS), static_cast<unsigned>(N),
                          static_cast<unsigned>(secuencia), static_cast<unsigned long>(analisis),
                          static_cast<unsigned long>(desbordes), static_cast<unsigned long>(tMaxUs),
                          static_cast<unsigned long>(receptor.pistas_descartadas));
        } else if (c == 'f') {
            pedirEspectro = true;
        } else if (c == 't') {
            // Onda cuadrada de 1000 Hz al 50 %: fundamental en 1000 Hz y armonicos
            // impares (3000, 5000, 7000 Hz), todos tonos que el receptor debe ver.
            tonoPrueba = !tonoPrueba;
            if (tonoPrueba) {
                ledcAttach(PIN_PRUEBA, F_PRUEBA, 8);
                ledcWrite(PIN_PRUEBA, 128);
            } else {
                ledcDetach(PIN_PRUEBA);
            }
            Serial.printf("\n#prueba tono=%s f=%lu Hz pin=%u\n", tonoPrueba ? "on" : "off",
                          static_cast<unsigned long>(F_PRUEBA), static_cast<unsigned>(PIN_PRUEBA));
        }
    }
}

// Analiza los ultimos N valores y envia el diagnostico (y el espectro si se pidio).
void analizar() {
    const uint32_t t0 = micros();
    receptor.analizar(ventana);
    const uint32_t t = micros() - t0;
    if (t > tMaxUs) tMaxUs = t;

    const size_t largo = trama::empaquetarDiagnostico(
        analisis, static_cast<uint16_t>(t > 65535 ? 65535 : t),
        receptor.validadas, static_cast<size_t>(receptor.n_validadas), salidaDiagnostico);
    Serial.write(salidaDiagnostico, largo);

    if (pedirEspectro) {
        pedirEspectro = false;
        const size_t l = trama::empaquetarEspectro(analisis, ventana, receptor.potencia, N,
                                                   salidaEspectro);
        Serial.write(salidaEspectro, l);
    }
    ++analisis;
}

// Cada muestra va a las dos rutas: la trama de muestras y la ventana de analisis.
void agregarMuestra(uint16_t codigo) {
    bloque[llenas++] = static_cast<int16_t>(codigo);
    if (llenas == N) {
        const size_t largo = trama::empaquetar(secuencia++, bloque, N, salidaMuestras);
        Serial.write(salidaMuestras, largo);
        llenas = 0;
    }

    // El analisis a cubre las muestras [512 a, 512 a + 1024): con la ventana llena
    // se analiza y se conservan las ultimas 512 para el siguiente (50 % de solapamiento).
    ventana[enVentana++] = codigo / 4095.0f;
    if (enVentana == N) {
        analizar();
        std::memmove(ventana, ventana + SALTO, (N - SALTO) * sizeof(float));
        enVentana = N - SALTO;
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
        agregarMuestra(r->type2.data);
    }
}
