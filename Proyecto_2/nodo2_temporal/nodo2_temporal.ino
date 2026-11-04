#include <Arduino.h>
#include <cmath>
#include "filtro_fir.h"
#include "coeficientes.h"

// Diagnostico local: no adquiere ADC ni recibe SPI ni reproduce audio por DAC.
// El impulso al final del primer bloque debe continuar en el bloque siguiente.
constexpr size_t N = 1024;
float entrada[N];
float salida[N];
temporal::FiltroFIR<temporal::MULTITONO.size()> filtro(temporal::MULTITONO);

void probarFiltro() {
    // Reiniciar permite repetir la prueba sin heredar muestras de otra captura.
    filtro.reiniciar();
    float errorMaximo = 0.0f;
    bool correcto = true;
    for (size_t bloque = 0; bloque < 2; ++bloque) {
        for (size_t i = 0; i < N; ++i) entrada[i] = 0.0f;
        if (bloque == 0) entrada[N - 1] = 1.0f;
        filtro.procesar(entrada, salida, N);

        // La respuesta a un impulso debe ser exactamente la secuencia de coeficientes.
        for (size_t i = 0; i < N; ++i) {
            const size_t muestra = bloque * N + i;
            float esperado = 0.0f;
            if (muestra >= N - 1 && muestra - (N - 1) < temporal::MULTITONO.size()) {
                esperado = temporal::MULTITONO[muestra - (N - 1)];
            }
            const float error = std::fabs(salida[i] - esperado);
            if (!std::isfinite(salida[i]) || error > 2e-6f) correcto = false;
            if (error > errorMaximo) errorMaximo = error;
        }
    }
    Serial.println("Diagnostico local FIR MULTITONO: fs de diseno 16000 Hz, orden 42.");
    Serial.printf("Impulso entre bloques: %s; error maximo=%.9g\n",
                  correcto ? "PASS" : "FAIL", static_cast<double>(errorMaximo));
    Serial.println("Esta prueba no mide tiempo real ni verifica ADC, SPI o DAC.");
}

void setup() {
    // El comando permite ejecutar despues de abrir el monitor, sin depender del arranque USB.
    Serial.begin(115200);
}

void loop() {
    // La prueba se inicia explicitamente y se puede repetir sin reiniciar la placa.
    if (Serial.available() && Serial.read() == 't') probarFiltro();
    delay(1);
}
