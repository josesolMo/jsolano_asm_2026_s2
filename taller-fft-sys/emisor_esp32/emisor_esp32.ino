#include <Arduino.h>
#include "esp_arduino_version.h"
#include "driver/dac_continuous.h"
#include "driver/dac_oneshot.h"
#include "senales.h"

// Fijar plataforma y formato impide transmitir accidentalmente muestras mal alineadas.
#if !CONFIG_IDF_TARGET_ESP32
#error "Seleccione ESP32 Dev Module (ESP-WROOM-32 clasico)."
#endif
#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(3, 3, 12)
#error "Este emisor requiere Arduino-ESP32 3.3.12."
#endif
#if !CONFIG_DAC_DMA_AUTO_16BIT_ALIGN
#error "Se requiere CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=1 para el buffer uint8_t."
#endif

// El buffer completo evita calculos de senos y normalizaciones durante la salida.
constexpr size_t BLOQUE = 4096;
constexpr size_t TOTAL = emisor::MUESTRAS + BLOQUE;
uint8_t muestras[TOTAL];
emisor::Modo seleccion = emisor::Modo::Seno1000;
dac_continuous_handle_t continuo = nullptr;
dac_oneshot_handle_t reposo = nullptr;
bool habilitado = false;
enum class EstadoEmisor { Reposo, Reproduccion, Error };
EstadoEmisor estadoActual = EstadoEmisor::Error;
size_t cargadas = 0;
uint32_t recortes = 0;
uint32_t errores = 0;
esp_err_t ultimoError = ESP_OK;

// La ISR solo registra el fin de la cadena; nunca calcula ni imprime muestras.
portMUX_TYPE cerrojo = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t finalesDma = 0;
volatile uint32_t convertidas = 0;
volatile uint32_t agotamientos = 0;
bool IRAM_ATTR alConvertir(dac_continuous_handle_t, const dac_event_data_t* evento, void*) {
    // Los bytes DMA incluyen dos bytes por codigo de 8 bits en este ESP32.
    portENTER_CRITICAL_ISR(&cerrojo);
    convertidas = convertidas + evento->write_bytes / 2;
    portEXIT_CRITICAL_ISR(&cerrojo);
    return false;
}
bool IRAM_ATTR alFinalizar(dac_continuous_handle_t, const dac_event_data_t*, void*) {
    portENTER_CRITICAL_ISR(&cerrojo);
    finalesDma = finalesDma + 1;
    if (convertidas < TOTAL) agotamientos = agotamientos + 1;
    portEXIT_CRITICAL_ISR(&cerrojo);
    return false;
}

// La copia protegida permite consultar el diagnostico sin carreras con la ISR.
void leerContadores(uint32_t& finales, uint32_t& convertidasCopia, uint32_t& fallos) {
    portENTER_CRITICAL(&cerrojo);
    finales = finalesDma;
    convertidasCopia = convertidas;
    fallos = agotamientos;
    portEXIT_CRITICAL(&cerrojo);
}

// Todo fallo de API queda disponible en el estado y detiene el avance normal.
bool comprobar(esp_err_t resultado) {
    if (resultado == ESP_OK) return true;
    ultimoError = resultado;
    ++errores;
    estadoActual = EstadoEmisor::Error;
    Serial.printf("Error DAC: %s\n", esp_err_to_name(resultado));
    return false;
}

// El modo directo mantiene 128 sin una cadena DMA que pueda sobrescribirlo.
bool centrar() {
    if (!reposo) {
        dac_oneshot_config_t config = {};
        config.chan_id = DAC_CHAN_0;
        if (!comprobar(dac_oneshot_new_channel(&config, &reposo))) return false;
    }
    return comprobar(dac_oneshot_output_voltage(reposo, emisor::CENTRO));
}

// Primero se cancela DMA y se liberan sus recursos; despues se fija el centro.
bool detener() {
    // Hasta confirmar todas las API no se puede afirmar que existe reposo.
    estadoActual = EstadoEmisor::Error;
    if (continuo) {
        if (habilitado) {
            if (!comprobar(dac_continuous_disable(continuo))) return false;
            habilitado = false;
        }
        if (!comprobar(dac_continuous_del_channels(continuo))) return false;
        continuo = nullptr;
    }
    if (!centrar()) return false;
    estadoActual = EstadoEmisor::Reposo;
    return true;
}

// Se intenta liberar recursos tras un fallo, pero la recuperacion exige otro comando.
void limpiarTrasError() {
    detener();
    estadoActual = EstadoEmisor::Error;
    Serial.println("Estado error; use p para recuperar reposo o i para reintentar inicio.");
}

// Dormir brevemente permite ejecutar las tareas idle durante los calculos previos.
void cederPreparacion() {
    vTaskDelay(1);
}

// La configuracion APLL acepta 16 kHz en el ESP32 clasico, a diferencia de PLL_D2.
void iniciar() {
    if (!detener()) return;
    Serial.println("Preparando 65536 muestras; API de codigo central 128 confirmada, sin medicion fisica.");
    const double pico = emisor::pico_global(seleccion, cederPreparacion);
    recortes = 0;
    for (size_t i = 0; i < TOTAL; i += BLOQUE) {
        emisor::generar_bloque(seleccion, i, muestras + i, BLOQUE, pico, recortes);
        // Ceder CPU durante la preparacion evita bloquear otras tareas del sistema.
        cederPreparacion();
    }
    if (!comprobar(dac_oneshot_del_channel(reposo))) return;
    reposo = nullptr;
    dac_continuous_config_t config = {};
    config.chan_mask = DAC_CHANNEL_MASK_CH0;
    config.desc_num = 8;
    config.buf_size = 2048;
    config.freq_hz = emisor::FS;
    config.offset = 0;
    config.clk_src = DAC_DIGI_CLK_SRC_APLL;
    config.chan_mode = DAC_CHANNEL_MODE_SIMUL;
    if (!comprobar(dac_continuous_new_channels(&config, &continuo))) {
        limpiarTrasError();
        return;
    }
    dac_event_callbacks_t callbacks = {};
    callbacks.on_convert_done = alConvertir;
    callbacks.on_stop = alFinalizar;
    if (!comprobar(dac_continuous_register_event_callback(continuo, &callbacks, nullptr))) {
        limpiarTrasError();
        return;
    }
    // Reiniciar la contabilidad junta evita mezclar cargas anteriores si falla enable.
    cargadas = 0;
    portENTER_CRITICAL(&cerrojo);
    finalesDma = 0;
    convertidas = 0;
    agotamientos = 0;
    portEXIT_CRITICAL(&cerrojo);
    if (!comprobar(dac_continuous_enable(continuo))) {
        limpiarTrasError();
        return;
    }
    habilitado = true;
    estadoActual = EstadoEmisor::Reproduccion;
}

// Los comandos son caracteres independientes; CR y LF se ignoran.
void ayuda() {
    Serial.println("1=1000 Hz; 2=2500 Hz; 3=multitono; 4=transitoria; 5=melodia");
    Serial.println("i=iniciar desde cero; p=parar y centrar; e=estado; ?=ayuda");
    Serial.println("Seleccionar otro modo detiene; luego pulse i. Todos duran 4.096 s.");
}

// La tasa real no es medida por software; se informa explicitamente esta limitacion.
void estado() {
    // Todos los contadores ISR se capturan juntos antes de imprimir.
    uint32_t finales, convertidasCopia, fallosDma;
    leerContadores(finales, convertidasCopia, fallosDma);
    Serial.printf("Modo %u: %s\n", static_cast<unsigned>(seleccion), emisor::descripcion(seleccion));
    Serial.printf("Estado: %s; fs nominal/configurada=%lu Hz; real sin medir (APLL)\n",
                  estadoActual == EstadoEmisor::Reposo ? "reposo" :
                  estadoActual == EstadoEmisor::Reproduccion ? "reproduccion" : "error",
                  static_cast<unsigned long>(emisor::FS));
    Serial.printf("DAC1 GPIO25, 8 bits, centro=128, excursion=80; cargadas=%u; convertidas DMA=%lu; total esperado=%u\n",
                  static_cast<unsigned>(cargadas), static_cast<unsigned long>(convertidasCopia),
                  static_cast<unsigned>(TOTAL));
    Serial.printf("Recortes=%lu; errores=%lu; ultimo=%s; fines DMA=%lu; agotamientos=%lu\n",
                  static_cast<unsigned long>(recortes), static_cast<unsigned long>(errores),
                  esp_err_to_name(ultimoError), static_cast<unsigned long>(finales),
                  static_cast<unsigned long>(fallosDma));
    Serial.printf("Recursos: continuo=%s; habilitado=%s; directo=%s. Contadores DMA sin medicion fisica.\n",
                  continuo ? "si" : "no", habilitado ? "si" : "no", reposo ? "si" : "no");
}

void setup() {
    // El emisor arranca detenido para que la reproduccion sea una accion explicita.
    Serial.begin(115200);
    detener();
    ayuda();
}

void loop() {
    // El control se atiende entre escrituras; no transmite muestras por serial.
    while (Serial.available()) {
        const char comando = Serial.read();
        if (comando >= '1' && comando <= '5') {
            if (detener()) {
                seleccion = static_cast<emisor::Modo>(comando - '0');
                Serial.println(emisor::descripcion(seleccion));
            }
        } else if (comando == 'i') iniciar();
        else if (comando == 'p') {
            if (detener()) Serial.println("Reposo confirmado por API; codigo 128, sin medicion fisica.");
        }
        else if (comando == 'e') estado();
        else if (comando == '?') ayuda();
        else if (comando != '\r' && comando != '\n' && comando != ' ') {
            Serial.println("Comando desconocido; use ?.");
        }
    }

    // Un fin anticipado indica agotamiento de DMA: se detiene sin ocultar el hueco.
    uint32_t finales, convertidasCopia, fallosDma;
    leerContadores(finales, convertidasCopia, fallosDma);
    if (estadoActual == EstadoEmisor::Reproduccion && finales != 0) {
        if (fallosDma != 0) {
            comprobar(ESP_ERR_INVALID_STATE);
            Serial.println("Error: DMA termino antes de convertir la secuencia completa.");
            limpiarTrasError();
        } else if (detener()) {
            Serial.println("Reproduccion finalizada; reposo confirmado por API, sin medicion fisica.");
        }
    }

    // La API copia al DMA; el reloj APLL, no este bucle, fija cada conversion.
    if (estadoActual == EstadoEmisor::Reproduccion && cargadas < TOTAL) {
        size_t escritas = 0;
        const esp_err_t resultado = dac_continuous_write(
            continuo, muestras + cargadas, BLOQUE, &escritas, 1000);
        cargadas += escritas;
        if (!comprobar(resultado) || escritas != BLOQUE) {
            if (resultado == ESP_OK) comprobar(ESP_FAIL);
            limpiarTrasError();
        }
    }

    // Dormir una tarea no temporiza las muestras y evita espera activa en reposo.
    vTaskDelay(1);
}
