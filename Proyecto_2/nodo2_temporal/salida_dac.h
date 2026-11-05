#pragma once
#include "esp_arduino_version.h"
#include "driver/dac_continuous.h"
#include "driver/dac_oneshot.h"
#include "conversion_dac.h"

// GPIO25 corresponde al DAC del ESP32 clasico, no al ESP32-S3 de adquisicion.
#if !CONFIG_IDF_TARGET_ESP32
#error "La salida GPIO25 requiere un ESP32 clasico. Seleccione ESP32 Dev Module."
#endif
#if ESP_ARDUINO_VERSION != ESP_ARDUINO_VERSION_VAL(3, 3, 12)
#error "Esta integracion se verifica con Arduino-ESP32 3.3.12."
#endif
#if !CONFIG_DAC_DMA_AUTO_16BIT_ALIGN
#error "Se requiere CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=1 para muestras uint8_t."
#endif

namespace diagnostico_dac {
// El buffer completo evita calculos de senos y normalizaciones durante la salida.
constexpr size_t BLOQUE = 1024;
// Una escritura de ocho bloques alimenta varios descriptores antes de volver al loop.
// Enviar solo 1024 muestras puede agotar el primer descriptor antes de la siguiente llamada.
constexpr size_t ENVIO = 8 * BLOQUE;
constexpr size_t UTILES = 65536;
constexpr size_t TOTAL = UTILES + BLOQUE;
uint8_t muestras[TOTAL];
salida_dac::Contadores conversion;
dac_continuous_handle_t continuo = nullptr;
dac_oneshot_handle_t reposo = nullptr;
bool habilitado = false;
enum class EstadoEmisor { Reposo, Reproduccion, Error };
EstadoEmisor estadoActual = EstadoEmisor::Error;
size_t cargadas = 0;
uint32_t recortes = 0;
uint32_t errores = 0;
esp_err_t ultimoError = ESP_OK;

// La ISR solo registra el fin de la cadena
portMUX_TYPE cerrojo = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t finalesDma = 0;
volatile uint32_t convertidas = 0;
volatile uint32_t agotamientos = 0;
size_t completadasAlCierre = 0;
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
    // on_stop confirma el fin de la cadena; la tarea evalua si toda la captura estaba cargada.
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
    return comprobar(dac_oneshot_output_voltage(reposo, salida_dac::CENTRO));
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
    conversion = {};
    filtro.reiniciar();
    // Diagnostico conocido: tono util de 1 kHz e interferencia de 2.5 kHz.
    // El indice absoluto conserva fase entre bloques; no se normaliza por bloque.
    for (size_t inicio = 0; inicio < TOTAL; inicio += BLOQUE) {
        for (size_t i = 0; i < BLOQUE; ++i) {
            const size_t n = inicio + i;
            float valor = 0.0f;
            if (n < UTILES) {
                const double t = static_cast<double>(n) / 16000.0;
                const size_t borde = n < UTILES / 2 ? n : UTILES - 1 - n;
                const double rampa = borde < 320 ? std::pow(std::sin(1.5707963267948966 * borde / 319), 2) : 1.0;
                valor = static_cast<float>(rampa * (.6 * std::sin(6.283185307179586 * 1000 * t)
                                                   + .3 * std::sin(6.283185307179586 * 2500 * t)));
            }
            entrada[i] = valor;
        }
        filtro.procesar(entrada, salida, BLOQUE);
        salida_dac::convertir(salida, muestras + inicio, BLOQUE, conversion);
        cederPreparacion();
    }
    // El bloque final contiene la cola FIR y termina en el codigo de reposo 128.
    recortes = conversion.recortes;
    if (!comprobar(dac_oneshot_del_channel(reposo))) return;
    reposo = nullptr;
    dac_continuous_config_t config = {};
    config.chan_mask = DAC_CHANNEL_MASK_CH0;
    config.desc_num = 8;
    config.buf_size = 2048;
    config.freq_hz = 16000;
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
    completadasAlCierre = 0;
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

// El estado informa progreso DMA y errores sin afirmar una tasa fisicamente medida.
void estado() {
    uint32_t finales, convertidasCopia, fallos;
    leerContadores(finales, convertidasCopia, fallos);
    Serial.printf("DAC GPIO25: fs nominal=16000 Hz (APLL); estado=%s\n",
                  estadoActual == EstadoEmisor::Reproduccion ? "reproduccion" :
                  estadoActual == EstadoEmisor::Reposo ? "reposo" : "error");
    Serial.printf("Centro=128; escala=60; cargadas=%u; conteo eventos DMA=%lu; esperadas=%u\n",
                  static_cast<unsigned>(cargadas), static_cast<unsigned long>(convertidasCopia),
                  static_cast<unsigned>(TOTAL));
    Serial.printf("Completadas al cierre de cadena=%u (confirmacion on_stop, no medicion fisica)\n",
                  static_cast<unsigned>(completadasAlCierre));
    Serial.printf("Recortes=%lu; no finitas=%lu; errores API=%lu; agotamientos=%lu; fines DMA=%lu\n",
                  static_cast<unsigned long>(recortes), static_cast<unsigned long>(conversion.noFinitas),
                  static_cast<unsigned long>(errores), static_cast<unsigned long>(fallos),
                  static_cast<unsigned long>(finales));
    Serial.printf("Ultimo error API: %s\n", esp_err_to_name(ultimoError));
}

// El bucle solo alimenta DMA; los instantes de conversion los fija el periferico.
void atender() {
    uint32_t finales, convertidasCopia, fallos;
    leerContadores(finales, convertidasCopia, fallos);
    if (estadoActual == EstadoEmisor::Reproduccion && finales != 0) {
        // No usar la suma de callbacks como prueba de perdida: el driver puede
        // reutilizar descriptores antes de notificar su longitud al callback.
        // Un cierre antes de cargar todo o varios cierres revela una discontinuidad.
        if (cargadas != TOTAL || finales != 1) {
            portENTER_CRITICAL(&cerrojo);
            agotamientos = agotamientos + 1;
            portEXIT_CRITICAL(&cerrojo);
            Serial.println("Error de continuidad: cierre DMA antes de completar una cadena unica.");
            limpiarTrasError();
        } else {
            completadasAlCierre = cargadas;
            if (!detener()) return;
            Serial.println("Reproduccion finalizada; DAC centrado en 128 por API.");
            estado();
        }
    }
    if (estadoActual == EstadoEmisor::Reproduccion && cargadas < TOTAL) {
        // Mantener reserva DMA es independiente del tamano de bloque usado por el FIR.
        const size_t pendientes = TOTAL - cargadas;
        const size_t cantidad = pendientes < ENVIO ? pendientes : ENVIO;
        size_t escritas = 0;
        const esp_err_t resultado = dac_continuous_write(
            continuo, muestras + cargadas, cantidad, &escritas, 1000);
        cargadas += escritas;
        if (!comprobar(resultado) || escritas != cantidad) {
            if (resultado == ESP_OK) comprobar(ESP_FAIL);
            limpiarTrasError();
        }
    }
}
}  // namespace diagnostico_dac
