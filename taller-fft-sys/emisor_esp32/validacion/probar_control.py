"""Compila el sketch real con dobles de API para probar errores sin una placa."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

# Los dobles solo simulan respuestas de API y serial; no simulan voltajes ni reloj.
MOCK = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <string>
#define CONFIG_IDF_TARGET_ESP32 1
#define CONFIG_DAC_DMA_AUTO_16BIT_ALIGN 1
#define ESP_ARDUINO_VERSION_VAL(a,b,c) ((a)*10000+(b)*100+(c))
#define ESP_ARDUINO_VERSION ESP_ARDUINO_VERSION_VAL(3,3,12)
#define IRAM_ATTR
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define portENTER_CRITICAL_ISR(x) ((void)(x))
#define portEXIT_CRITICAL_ISR(x) ((void)(x))
using portMUX_TYPE = int;
using esp_err_t = int;
constexpr int ESP_OK=0, ESP_FAIL=-1, ESP_ERR_INVALID_STATE=2;
constexpr int DAC_CHAN_0=0, DAC_CHANNEL_MASK_CH0=1;
constexpr int DAC_DIGI_CLK_SRC_APLL=1, DAC_CHANNEL_MODE_SIMUL=0;
using dac_continuous_handle_t = void*;
using dac_oneshot_handle_t = void*;
struct dac_event_data_t { size_t write_bytes; };
using Callback = bool (*)(void*,const dac_event_data_t*,void*);
struct dac_event_callbacks_t { Callback on_convert_done; Callback on_stop; };
struct dac_oneshot_config_t { int chan_id; };
struct dac_continuous_config_t {
    int chan_mask,desc_num; size_t buf_size; uint32_t freq_hz;
    int offset,clk_src,chan_mode;
};
std::string fallo;
int respuesta(const char* nombre) { return fallo == nombre ? ESP_FAIL : ESP_OK; }
const char* esp_err_to_name(int e) { return e == ESP_OK ? "ESP_OK" : "FALLO_SIMULADO"; }
void vTaskDelay(int) {}
struct SerialFalso {
    std::string entrada, salida;
    void begin(int) {}
    int available() { return entrada.size(); }
    char read() { char c=entrada[0]; entrada.erase(0,1); return c; }
    void println(const char* s) { salida += std::string(s)+"\n"; }
    template<class... Args> void printf(const char* f, Args... args) {
        char b[1024]; std::snprintf(b,sizeof(b),f,args...); salida += b;
    }
} Serial;
int dac_oneshot_new_channel(const dac_oneshot_config_t*,void** h) {
    int r=respuesta("directo_nuevo"); if (!r) *h=reinterpret_cast<void*>(1); return r;
}
int dac_continuous_new_channels(const dac_continuous_config_t*,void** h) {
    int r=respuesta("continuo_nuevo"); if (!r) *h=reinterpret_cast<void*>(2); return r;
}
int dac_oneshot_output_voltage(void*,uint8_t) { return respuesta("centrar"); }
int dac_oneshot_del_channel(void*) { return respuesta("directo_borrar"); }
int dac_continuous_disable(void*) { return respuesta("deshabilitar"); }
int dac_continuous_del_channels(void*) { return respuesta("continuo_borrar"); }
int dac_continuous_enable(void*) { return respuesta("habilitar"); }
int dac_continuous_register_event_callback(void*,const dac_event_callbacks_t*,void*) {
    return respuesta("callbacks");
}
int dac_continuous_write(void*,uint8_t*,size_t n,size_t* escritas,int) {
    *escritas = fallo == "parcial" ? n/2 : n;
    return respuesta("escribir");
}
'''

# Las aserciones observan el estado y los mensajes del .ino incluido, sin copiar su control.
PRUEBAS = r'''
#include <cassert>
#include "emisor_esp32.ino"
void reiniciar() {
    fallo.clear(); continuo=nullptr; reposo=nullptr; habilitado=false;
    estadoActual=EstadoEmisor::Error; errores=0; ultimoError=ESP_OK;
    finalesDma=0; convertidas=0; agotamientos=0; cargadas=0;
    seleccion=emisor::Modo::Seno1000; Serial.entrada.clear(); Serial.salida.clear();
    assert(detener());
}
void comando(char c) { Serial.entrada=c; Serial.salida.clear(); loop(); }
int main() {
    // La parada solo confirma las operaciones completas y conserva cada recurso fallido.
    for (const char* nombre : {"deshabilitar","continuo_borrar","directo_nuevo","centrar"}) {
        reiniciar(); iniciar(); assert(estadoActual==EstadoEmisor::Reproduccion);
        fallo=nombre; comando('p'); assert(estadoActual==EstadoEmisor::Error);
        assert(Serial.salida.find("Reposo confirmado")==std::string::npos);
        if (fallo=="deshabilitar") assert(continuo && habilitado);
        if (fallo=="continuo_borrar") assert(continuo && !habilitado);
        if (fallo=="centrar") assert(reposo && !continuo);
        assert(errores>0); fallo.clear(); comando('p');
        assert(estadoActual==EstadoEmisor::Reposo && reposo && !continuo && !habilitado);
        assert(errores>0 && ultimoError!=ESP_OK);
    }
    // Inicio y reinicio no avanzan cuando falla un paso previo o la reserva de recursos.
    for (const char* nombre : {"centrar","directo_borrar","continuo_nuevo","callbacks","habilitar"}) {
        reiniciar(); fallo=nombre; iniciar(); assert(estadoActual==EstadoEmisor::Error);
        assert(!habilitado); if (fallo=="directo_borrar") assert(reposo);
        fallo.clear(); iniciar(); assert(estadoActual==EstadoEmisor::Reproduccion);
    }
    reiniciar(); iniciar(); fallo="deshabilitar"; iniciar();
    assert(estadoActual==EstadoEmisor::Error && continuo && habilitado);
    comando('3'); assert(seleccion==emisor::Modo::Seno1000);
    fallo.clear(); comando('3'); assert(seleccion==emisor::Modo::Multitono);
    // La finalizacion automatica tampoco confirma reposo si falla su limpieza.
    for (const char* nombre : {"", "deshabilitar", "continuo_borrar", "centrar"}) {
        reiniciar(); iniciar(); cargadas=TOTAL; convertidas=TOTAL; finalesDma=1;
        fallo=nombre; Serial.salida.clear(); loop();
        assert((estadoActual==EstadoEmisor::Reposo)==fallo.empty());
        assert((Serial.salida.find("Reproduccion finalizada")!=std::string::npos)==fallo.empty());
    }
    // Los fallos de escritura y agotamientos permanecen visibles aun con limpieza exitosa.
    for (const char* nombre : {"escribir", "parcial", "agotamiento"}) {
        reiniciar(); iniciar(); fallo=nombre;
        if (fallo=="agotamiento") { finalesDma=1; agotamientos=1; }
        loop(); assert(estadoActual==EstadoEmisor::Error && errores>0);
    }
    reiniciar(); cargadas=4096; convertidas=1024; finalesDma=2; agotamientos=1;
    comando('e'); assert(Serial.salida.find("cargadas=4096; convertidas DMA=1024; total esperado=69632")!=std::string::npos);
    std::puts("Control OK: parada, inicio, reinicio, seleccion, fin, recuperacion y contadores con API simuladas.");
}
'''


def main():
    # Todo producto de compilacion queda en un directorio temporal, fuera del sketch.
    carpeta = Path(__file__).resolve().parents[1]
    compilador = shutil.which(os.environ.get("CXX", "g++"))
    if not compilador:
        raise RuntimeError("Falta g++ o CXX")
    with tempfile.TemporaryDirectory(prefix="emisor_control_") as temporal:
        raiz = Path(temporal)
        (raiz / "driver").mkdir()
        (raiz / "Arduino.h").write_text(MOCK, encoding="utf-8")
        for nombre in ("esp_arduino_version.h", "driver/dac_continuous.h", "driver/dac_oneshot.h"):
            (raiz / nombre).write_text('#include "Arduino.h"\n', encoding="utf-8")
        (raiz / "prueba.cpp").write_text(PRUEBAS, encoding="utf-8")
        ejecutable = raiz / ("prueba.exe" if os.name == "nt" else "prueba")
        subprocess.run([compilador, "-std=c++17", "-O2", "-Wall", "-Wextra",
                        "-I", str(raiz), "-I", str(carpeta), str(raiz / "prueba.cpp"),
                        "-o", str(ejecutable)], check=True)
        subprocess.run([str(ejecutable)], check=True)


if __name__ == "__main__":
    main()
