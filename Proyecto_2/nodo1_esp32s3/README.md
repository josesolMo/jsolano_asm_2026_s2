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
| 5. Parámetros sin recompilar, tiempos y memoria | Comandos listos; tiempo medido en la placa (~5 ms de 32 ms); fs real del ADC 16 028 Hz |

## Archivos

| Archivo | Qué hace |
|---|---|
| `nodo1_esp32s3.ino` | Firmware: ADC1 canal 3 (GPIO4) en modo continuo por DMA; tramas de muestras cada 64 ms, análisis y trama de diagnóstico cada 32 ms. Comandos en la tabla de abajo. |
| `receptor.h` | Receptor en C++: FFT radix-2 propia (float32), piso de ruido (percentil 25 de 31 bins), criterios C1–C5 y rango de reporte de 40 dB. Traducción de `receptor_fft/src/receptor_fft.py`. |
| `trama.h` | Tramas: muestras `A5 5A` (formato de `full_system/sistema_ruido/link.py`), diagnóstico `A5 5B` y espectro `A5 5C`, todas con CRC-16/CCITT. |
| `captura_nodo1.py` | PC (paso 1): recibe las muestras y corre el receptor de Python. |
| `validar_nodo1.py` | PC (pasos 2 y 3): compara la FFT de la placa con numpy y sus frecuencias con el receptor de Python, bloque a bloque; resume lo detectado y el tiempo de análisis. |
| `validacion/probar_trama.py` | Sin placa: compila `trama.h` y comprueba que sus bytes son idénticos a `link.py`. |
| `validacion/probar_receptor.py` | Sin placa: compila `receptor.h` y lo compara con Python en 7 señales (868/868 bloques idénticos). |

Las pruebas de `validacion/` necesitan `g++`; los scripts de PC necesitan numpy y
scipy, y `pyserial` para leer la placa.

## Comandos por el monitor serial

| Comando | Qué hace |
|---|---|
| `e` | Estado: tramas, análisis, desbordes, peor tiempo de análisis |
| `p` | Parámetros actuales |
| `u20` + Enter | Umbral C1 en dB sobre el piso (5 a 40) |
| `b340,1420` + Enter | Banda útil en Hz (C5) |
| `r40` + Enter | Rango de reporte: no se reporta lo que esté más de N dB bajo el tono más fuerte (10 a 80) |
| `t` | Enciende/apaga el tono de prueba de 1000 Hz en GPIO5 (jumper GPIO5 → GPIO4) |
| `f` | Envía el espectro del próximo bloque (lo usa `validar_nodo1.py`) |

`validar_nodo1.py` compara contra Python con los parámetros por defecto: si se cambian,
reinicie la placa (RST) antes de validar.

## Compilar y cargar (Arduino IDE)

- Paquete **Arduino-ESP32 3.3.12** (el mismo del emisor). Placa: **ESP32S3 Dev Module**.
- Flash Size: 16MB. PSRAM: OPI PSRAM.
- Puerto con convertidor USB-serial (marcado *UART*): *USB CDC On Boot: Disabled*.

## Uso

Desde la carpeta que contiene `nodo1_esp32s3/`, `full_system/` y `receptor_fft/`
(cerrar antes el monitor serial del IDE):

    python nodo1_esp32s3/validar_nodo1.py --puerto COM6 --segundos 4 --tono
    python nodo1_esp32s3/validar_nodo1.py --puerto COM6 --segundos 4 --modo 3

`--tono` usa el tono de 1000 Hz de la propia placa (jumper GPIO5 → GPIO4) y además
estima la frecuencia de muestreo real del ADC.

`--modo` (opcional) es el modo del emisor para comparar con su tabla
(1 = 1000 Hz, 2 = 2500 Hz, 3 = 440/880/1320 Hz, 4 = transitoria de 600 Hz).
Las capturas se guardan en `capturas/` (ignorada por git).