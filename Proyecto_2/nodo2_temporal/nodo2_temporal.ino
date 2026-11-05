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

// Consumir la salida impide que una optimizacion elimine el trabajo medido.
volatile float comprobacionTiempo = 0.0f;
constexpr uint32_t FS = 16000;
constexpr uint32_t PRESUPUESTO_US = N * 1000000UL / FS;
constexpr size_t REPETICIONES = 200;

template <size_t T>
void medirPerfil(const char* nombre, const std::array<float, T>& coeficientes) {
    temporal::FiltroFIR<T> filtroMedido(coeficientes);
    // Calentar codigo y estado antes de recoger tiempos evita medir solo el arranque.
    for (size_t i = 0; i < 10; ++i) {
        filtroMedido.procesar(entrada, salida, N);
        comprobacionTiempo = salida[N - 1];
        delay(1);
    }

    uint32_t minimo = UINT32_MAX;
    uint32_t maximo = 0;
    uint64_t total = 0;
    size_t excedidos = 0;
    float suma = 0.0f;
    bool finitos = true;
    for (size_t repeticion = 0; repeticion < REPETICIONES; ++repeticion) {
        // Solo se cronometra el filtro; se mantienen las interrupciones habilitadas.
        const uint32_t inicio = micros();
        filtroMedido.procesar(entrada, salida, N);
        const uint32_t duracion = static_cast<uint32_t>(micros() - inicio);
        if (duracion < minimo) minimo = duracion;
        if (duracion > maximo) maximo = duracion;
        total += duracion;
        if (duracion >= PRESUPUESTO_US) ++excedidos;

        // Verificacion y cesion al sistema quedan fuera del intervalo cronometrado.
        for (size_t i = 0; i < N; ++i) {
            finitos = finitos && std::isfinite(salida[i]);
            suma += salida[i];
        }
        comprobacionTiempo = suma;
        delay(1);
    }

    // La fraccion del presupuesto es tiempo de pared, no una medicion de uso de CPU.
    const double promedio = static_cast<double>(total) / REPETICIONES;
    Serial.printf("%s: %u coeficientes, %u bloques de %u muestras\n", nombre,
                  static_cast<unsigned>(T), static_cast<unsigned>(REPETICIONES),
                  static_cast<unsigned>(N));
    Serial.printf("Tiempo us: min=%lu; promedio=%.2f; max=%lu\n",
                  static_cast<unsigned long>(minimo), promedio,
                  static_cast<unsigned long>(maximo));
    Serial.printf("Presupuesto=%lu us; uso promedio=%.2f %%; uso maximo=%.2f %%\n",
                  static_cast<unsigned long>(PRESUPUESTO_US),
                  100.0 * promedio / PRESUPUESTO_US, 100.0 * maximo / PRESUPUESTO_US);
    Serial.printf("Bloques sin margen (>= presupuesto): %u; salida finita: %s; suma=%.6f\n",
                  static_cast<unsigned>(excedidos), finitos ? "SI" : "NO",
                  static_cast<double>(suma));
}

void medirTiempo() {
    // Ruido determinista de pico <= 0.5 ejercita el filtro sin generar senales al cronometrar.
    uint32_t estado = 2026;
    for (size_t i = 0; i < N; ++i) {
        estado = 1664525UL * estado + 1013904223UL;
        entrada[i] = static_cast<float>(estado >> 8) / 16777216.0f - 0.5f;
    }
    Serial.println("Diagnostico de tiempo FIR: fs=16000 Hz, N=1024, presupuesto=64000 us.");
    Serial.println("200 mediciones por perfil; 10 bloques previos de calentamiento.");
    // Esperar el envio evita que los mensajes iniciales se acumulen durante la medicion.
    Serial.flush();
    medirPerfil("TONO", temporal::TONO);
    Serial.flush();
    medirPerfil("MULTITONO", temporal::MULTITONO);
    Serial.flush();
    medirPerfil("AUDIO", temporal::AUDIO);
    Serial.flush();
    medirPerfil("TRANSITORIA", temporal::TRANSITORIA);
    Serial.println("No incluye ADC, SPI, conversion a 8 bits ni DAC; no garantiza tiempo real integrado.");
}

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
    if (Serial.available()) {
        const char comando = Serial.read();
        if (comando == 't') probarFiltro();
        else if (comando == 'm') medirTiempo();
        else if (comando == '?') Serial.println("t=probar impulso; m=medir tiempo de los cuatro perfiles");
    }
    delay(1);
}
