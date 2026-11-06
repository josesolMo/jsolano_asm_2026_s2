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
// Enlace al nodo 3 (UART1, TX = GPIO17 -> GPIO16 del nodo 3, GND comun, 921600 baud):
// por cada bloque de 1024 muestras envia la trama PCM, la de parametros (frecuencias
// validadas, tipo 0x02) y la del piso de ruido (tipo 0x03, piso_nodo3.h).
//
// Comandos por el monitor serial (una letra):
//   e  estado (texto)     f  enviar el espectro del proximo bloque (validar la FFT)
//   t  encender/apagar un tono de prueba de 1000 Hz (PWM) en GPIO5; con un jumper
//      GPIO5 -> GPIO4 la placa se mide a si misma sin otro equipo
// Si hay muestras recortadas (saturacion del ADC) avisa con "#saturacion ..." (max. 1 por segundo).
// Parametros sin recompilar (terminar la linea con Enter):
//   p              mostrar los parametros actuales
//   u20            umbral C1 en dB sobre el piso (5 a 40)
//   b340,1420      banda util en Hz (C5): lo que cae dentro no se reporta como ruido
//   r40            rango de reporte en dB bajo el tono mas fuerte (10 a 80)
//   h100           pasa altos de adquisicion en Hz (0 = apagado, el defecto; 0 a 400)
// Grabar y reproducir (la salida limpia puede sonar despues, sin mezclarse con el
// sonido contaminado que entra al microfono):
//   g5             graba 5 s (1 a 30) en la PSRAM; mientras graba no envia nada al nodo 3
//   x              reproduce lo grabado: reemplaza al microfono en todo el sistema
//   n              enciende/apaga el envio al nodo 3 (en vivo)
//   s              enciende/apaga las tramas binarias por USB (para leer el monitor; el
//                  enlace al nodo 3 sigue igual; validar_nodo1.py las necesita encendidas)
//
// En la PC: python nodo1_esp32s3/validar_nodo1.py --puerto COMx

#include <Arduino.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "esp_adc/adc_continuous.h"
#include "trama.h"
#include "receptor.h"
#include "piso_nodo3.h"
#include "pasa_altos.h"

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
constexpr int PIN_TX_NODO3 = 17;        // UART1 TX -> GPIO16 (RX2) del nodo 3
constexpr int PIN_RX_NODO3 = 18;        // UART1 RX (no se usa; el nodo 3 no responde)
constexpr uint32_t BAUD_NODO3 = 921600; // igual que config.h del nodo 3
constexpr uint32_t MAX_SEGUNDOS = 30;   // grabacion: 30 s x 16 kHz x 2 B = 960 KB de PSRAM
// Saturacion: codigos pegados a los extremos del ADC (0 V o ~3.1 V con 12 dB de atenuacion).
// Una muestra recortada agrega armonicos que no estan en el sonido y el filtro no puede quitar.
constexpr float HPF_DEFECTO_HZ = 0.0f;   // apagado: en placa agrego chasquidos audibles sin mejorar P1 (h100 lo enciende)
constexpr uint16_t SAT_BAJO = 8;
constexpr uint16_t SAT_ALTO = 4087;

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
nodo1::PasaAltos hpf;                   // acondicionamiento: quita DC y graves antes de todo
uint32_t saturadas = 0;                 // muestras recortadas desde el arranque
uint32_t saturadasBloque = 0;           // en el bloque PCM en curso
uint32_t ultimoAvisoMs = 0;

// Enlace al nodo 3.
nodo1::PisoNodo3 piso3;
int16_t nudos[nodo1::PisoNodo3::NUDOS];
uint8_t salidaParams[trama::tamanoParams(nodo1::MAX_REPORTADAS)];
uint8_t salidaPiso[trama::tamanoPiso(nodo1::PisoNodo3::NUDOS)];
bool enlaceNodo3 = true;
bool usbBinario = true;                 // tramas de muestras/diagnostico/espectro por USB
uint32_t enviadosNodo3 = 0;             // bloques enviados al nodo 3

// Grabacion en PSRAM: el ADC sigue marcando el ritmo; al reproducir, cada muestra del
// ADC se reemplaza por la grabada, asi todo el sistema (analisis, PC, nodo 3) la ve igual.
enum class Fuente : uint8_t { EnVivo, Grabando, Reproduciendo };
Fuente fuente = Fuente::EnVivo;
int16_t* grabacion = nullptr;
size_t grabadas = 0, objetivo = 0, posicion = 0;

bool enviarAlNodo3() { return enlaceNodo3 && fuente != Fuente::Grabando; }

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
    Serial1.setTxBufferSize(4096);      // > 1 bloque completo (PCM + PARAMS + PISO = ~2.2 KB)
    Serial1.begin(BAUD_NODO3, SERIAL_8N1, PIN_RX_NODO3, PIN_TX_NODO3);
    hpf.configurar(HPF_DEFECTO_HZ, FS);
    iniciarAdc();
}

void iniciarGrabacion(float segundos) {
    if (!(segundos >= 1.0f && segundos <= MAX_SEGUNDOS)) {
        Serial.printf("\n#error grabacion de 1 a %lu s\n", static_cast<unsigned long>(MAX_SEGUNDOS));
        return;
    }
    if (!grabacion) {
        grabacion = static_cast<int16_t*>(ps_malloc(MAX_SEGUNDOS * FS * sizeof(int16_t)));
        if (!grabacion) {
            Serial.println("\n#error sin PSRAM (Herramientas > PSRAM: OPI PSRAM)");
            return;
        }
    }
    objetivo = static_cast<size_t>(segundos * FS);
    grabadas = 0;
    fuente = Fuente::Grabando;
    Serial.printf("\n#grabando %.1f s (sin envio al nodo 3)\n", segundos);
}

void reproducir() {
    if (!grabacion || grabadas == 0 || fuente == Fuente::Grabando) {
        Serial.println("\n#error no hay grabacion lista (use g<segundos> primero)");
        return;
    }
    posicion = 0;
    fuente = Fuente::Reproduciendo;
    Serial.printf("\n#reproduciendo %.1f s\n", static_cast<float>(grabadas) / FS);
}

void mostrarParametros() {
    Serial.printf("\n#param umbral=%.1f dB banda=%.0f-%.0f Hz rango=%.1f dB hpf=%.0f Hz\n",
                  receptor.umbral_db, receptor.banda_lo, receptor.banda_hi, receptor.rango_db, hpf.fc());
}

// Aplica un comando con valor ("u20", "b340,1420", "r40"); rechaza valores fuera de rango
// para que un error de tipeo en la defensa no deje el nodo en un estado absurdo.
void aplicarParametro(char comando, const char* valor) {
    if (comando == 'u') {
        const float v = atof(valor);
        if (v >= 5.0f && v <= 40.0f) receptor.umbral_db = v;
        else Serial.printf("\n#error umbral fuera de 5..40 dB: %s\n", valor);
    } else if (comando == 'r') {
        const float v = atof(valor);
        if (v >= 10.0f && v <= 80.0f) receptor.rango_db = v;
        else Serial.printf("\n#error rango fuera de 10..80 dB: %s\n", valor);
    } else if (comando == 'b') {
        float lo = 0.0f, hi = 0.0f;
        if (sscanf(valor, "%f,%f", &lo, &hi) == 2 && lo >= 0.0f && hi <= FS / 2.0f && lo < hi) {
            receptor.banda_lo = lo;
            receptor.banda_hi = hi;
        } else {
            Serial.printf("\n#error banda invalida (use b<lo>,<hi> en Hz): %s\n", valor);
        }
    } else if (comando == 'h') {
        const float v = atof(valor);
        if (v >= 0.0f && v <= 400.0f) hpf.configurar(v, FS);
        else Serial.printf("\n#error pasa altos fuera de 0..400 Hz: %s\n", valor);
    } else if (comando == 'g') {
        iniciarGrabacion(atof(valor));
        return;
    }
    mostrarParametros();
}

// Los comandos se atienden entre lecturas; el script de la PC ignora el texto.
// e, f, t y p actuan al instante; u, b y r esperan su valor hasta el Enter.
char comandoPendiente = 0;
char valorPendiente[24];
size_t largoValor = 0;

void atenderComandos() {
    while (Serial.available()) {
        const char c = Serial.read();
        if (comandoPendiente) {
            if (c == '\n' || c == '\r') {
                valorPendiente[largoValor] = '\0';
                aplicarParametro(comandoPendiente, valorPendiente);
                comandoPendiente = 0;
            } else if (largoValor < sizeof(valorPendiente) - 1) {
                valorPendiente[largoValor++] = c;
            }
            continue;
        }
        if (c == 'u' || c == 'b' || c == 'r' || c == 'g' || c == 'h') {
            comandoPendiente = c;
            largoValor = 0;
        } else if (c == 'p') {
            mostrarParametros();
        } else if (c == 'e') {
            Serial.printf("\n#estado fs=%lu N=%u tramas=%u analisis=%lu desbordes=%lu "
                          "t_max_us=%lu pistas_descartadas=%lu\n",
                          static_cast<unsigned long>(FS), static_cast<unsigned>(N),
                          static_cast<unsigned>(secuencia), static_cast<unsigned long>(analisis),
                          static_cast<unsigned long>(desbordes), static_cast<unsigned long>(tMaxUs),
                          static_cast<unsigned long>(receptor.pistas_descartadas));
            Serial.printf("#saturacion muestras=%lu (%.3f %% del total)\n", static_cast<unsigned long>(saturadas),
                          (secuencia || llenas) ? 100.0 * saturadas / (static_cast<double>(secuencia) * N + llenas) : 0.0);
            Serial.printf("#nodo3 envio=%s bloques=%lu fuente=%s grabacion=%.1f s usb_tramas=%s\n",
                          enlaceNodo3 ? "on" : "off", static_cast<unsigned long>(enviadosNodo3),
                          fuente == Fuente::EnVivo ? "vivo" : fuente == Fuente::Grabando ? "grabando" : "reproduciendo",
                          static_cast<float>(grabadas) / FS, usbBinario ? "on" : "off");
        } else if (c == 'x') {
            reproducir();
        } else if (c == 's') {
            usbBinario = !usbBinario;
            Serial.printf("\n#usb tramas=%s (s para cambiar)\n", usbBinario ? "on" : "off");
        } else if (c == 'n') {
            enlaceNodo3 = !enlaceNodo3;
            Serial.printf("\n#nodo3 envio=%s\n", enlaceNodo3 ? "on" : "off");
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
    if (usbBinario) Serial.write(salidaDiagnostico, largo);

    if (pedirEspectro && usbBinario) {
        pedirEspectro = false;
        const size_t l = trama::empaquetarEspectro(analisis, ventana, receptor.potencia, N,
                                                   salidaEspectro);
        Serial.write(salidaEspectro, l);
    }

    // Los analisis pares cubren exactamente el bloque PCM que se acaba de enviar
    // (analisis 2s = muestras [1024 s, 1024 s + 1024)): el piso se actualiza una vez por
    // bloque, igual que el estimador local del nodo 3, y las dos tramas siguen a su PCM.
    if (analisis % 2 == 0) {
        piso3.actualizar(receptor.potencia);
        if (enviarAlNodo3()) {
            const uint8_t sec = static_cast<uint8_t>((secuencia - 1) & 0xFF);
            Serial1.write(salidaParams, trama::empaquetarParams(
                sec, receptor.validadas, static_cast<size_t>(receptor.n_validadas), salidaParams));
            piso3.nudos(nudos);
            Serial1.write(salidaPiso, trama::empaquetarPiso(
                sec, nodo1::PisoNodo3::DECIM, nudos, nodo1::PisoNodo3::NUDOS, salidaPiso));
        }
    }
    ++analisis;
}

// Cada muestra va a las dos rutas: la trama de muestras y la ventana de analisis.
void agregarMuestra(uint16_t crudo) {
    if (crudo <= SAT_BAJO || crudo >= SAT_ALTO) {   // la saturacion se mide en el codigo crudo
        ++saturadas;
        ++saturadasBloque;
    }
    // Pasa altos alrededor del centro del ADC; el resultado sigue en "codigos" (0..4095 nominal)
    // para que la trama PCM, el analisis y el nodo 3 no cambien de formato.
    float v = hpf.procesar(static_cast<float>(crudo) - 2048.0f) + 2048.0f;
    if (v < -32768.0f) v = -32768.0f;
    if (v > 32767.0f) v = 32767.0f;
    const int16_t codigo = static_cast<int16_t>(lrintf(v));
    bloque[llenas++] = codigo;
    if (llenas == N) {
        // Aviso como maximo una vez por segundo, para no llenar el monitor.
        if (saturadasBloque > 0 && millis() - ultimoAvisoMs >= 1000) {
            ultimoAvisoMs = millis();
            Serial.printf("\n#saturacion %lu de %u muestras recortadas en el bloque %u: baje la ganancia "
                          "del MAX4466 o aleje el microfono\n", static_cast<unsigned long>(saturadasBloque),
                          static_cast<unsigned>(N), static_cast<unsigned>(secuencia));
        }
        saturadasBloque = 0;
        const size_t largo = trama::empaquetar(secuencia++, bloque, N, salidaMuestras);
        if (usbBinario) Serial.write(salidaMuestras, largo);
        if (enviarAlNodo3()) {
            Serial1.write(salidaMuestras, largo);
            ++enviadosNodo3;
        }
        llenas = 0;
    }

    // El analisis a cubre las muestras [512 a, 512 a + 1024): con la ventana llena
    // se analiza y se conservan las ultimas 512 para el siguiente (50 % de solapamiento).
    ventana[enVentana++] = static_cast<float>(codigo) / 4095.0f;
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
        uint16_t codigo = r->type2.data;
        if (fuente == Fuente::Grabando) {
            grabacion[grabadas++] = static_cast<int16_t>(codigo);
            if (grabadas == objetivo) {
                fuente = Fuente::EnVivo;
                Serial.printf("\n#grabacion lista: %.1f s (x para reproducir)\n",
                              static_cast<float>(grabadas) / FS);
            }
        } else if (fuente == Fuente::Reproduciendo) {
            codigo = static_cast<uint16_t>(grabacion[posicion++]);
            if (posicion == grabadas) {
                fuente = Fuente::EnVivo;
                Serial.println("\n#fin de la reproduccion");
            }
        }
        agregarMuestra(codigo);
    }
}
