# Nodo 1 — Adquisición y análisis (ESP32-S3)

Captura la señal con el ADC a 16 kHz y, cada 32 ms, analiza los últimos 1024
valores (50 % de solapamiento): quita el DC, aplica Hann, calcula la FFT, estima el
piso de ruido y valida frecuencias con los criterios C1–C5 del receptor. Envía por
el puerto serial las muestras y el diagnóstico de cada bloque.

| Paso | Estado |
|---|---|
| 1. Captura a 16 kHz y tramas con CRC | Listo, probado en la placa |
| 2. DC + Hann + FFT en la placa | Listo; se valida contra numpy con `validar_nodo1.py` |
| 3. Receptor (C1–C5) en la placa | Listo; se valida contra Python con `validar_nodo1.py` |
| 4. Enlace SPI con los nodos 2 y 3 | Pendiente |
| 5. Parámetros sin recompilar, tiempos y memoria | Tiempo por bloque ya medido; resto pendiente |

## Archivos

| Archivo | Qué hace |
|---|---|
| `nodo1_esp32s3.ino` | Firmware: ADC1 canal 3 (GPIO4) en modo continuo por DMA; tramas de muestras cada 64 ms, análisis y trama de diagnóstico cada 32 ms. Comandos: `e` estado, `f` enviar el espectro del próximo bloque. |
| `receptor.h` | Receptor en C++: FFT radix-2 propia (float32), piso de ruido (percentil 25 de 31 bins), criterios C1–C5. Traducción de `receptor_fft/src/receptor_fft.py`. |
| `trama.h` | Tramas: muestras `A5 5A` (formato de `full_system/sistema_ruido/link.py`), diagnóstico `A5 5B` y espectro `A5 5C`, todas con CRC-16/CCITT. |
| `captura_nodo1.py` | PC (paso 1): recibe las muestras y corre el receptor de Python. |
| `validar_nodo1.py` | PC (pasos 2 y 3): compara la FFT de la placa con numpy y sus frecuencias con el receptor de Python, bloque a bloque; resume lo detectado y el tiempo de análisis. |
| `validacion/probar_trama.py` | Sin placa: compila `trama.h` y comprueba que sus bytes son idénticos a `link.py`. |
| `validacion/probar_receptor.py` | Sin placa: compila `receptor.h` y lo compara con Python en 7 señales (868/868 bloques idénticos). |

Las pruebas de `validacion/` necesitan `g++`; los scripts de PC necesitan numpy y
scipy, y `pyserial` para leer la placa.

## Compilar y cargar (Arduino IDE)

- Paquete **Arduino-ESP32 3.3.12** (el mismo del emisor). Placa: **ESP32S3 Dev Module**.
- Flash Size: 16MB. PSRAM: OPI PSRAM.
- Puerto con convertidor USB-serial (marcado *UART*): *USB CDC On Boot: Disabled*.

## Uso

Desde la carpeta que contiene `nodo1_esp32s3/`, `full_system/` y `receptor_fft/`
(cerrar antes el monitor serial del IDE):

    python nodo1_esp32s3/validar_nodo1.py --puerto COM6 --segundos 4
    python nodo1_esp32s3/validar_nodo1.py --puerto COM6 --segundos 4 --modo 3

`--modo` (opcional) es el modo del emisor para comparar con su tabla
(1 = 1000 Hz, 2 = 2500 Hz, 3 = 440/880/1320 Hz, 4 = transitoria de 600 Hz).
Las capturas se guardan en `capturas/` (ignorada por git).