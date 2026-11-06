# Nodo 1 — Adquisición y análisis (ESP32-S3)

Captura la señal con el ADC a 16 kHz y, cada 32 ms, analiza los últimos 1024
valores (50 % de solapamiento): quita el DC, aplica Hann, calcula la FFT, estima el
piso de ruido y valida frecuencias con los criterios C1–C5 del receptor. Envía por
el puerto serial (USB) las muestras y el diagnóstico de cada bloque, y por la UART1
(GPIO17) las muestras, las frecuencias validadas y el piso de ruido al nodo 3.

| Paso | Estado |
|---|---|
| 1. Captura a 16 kHz y tramas con CRC | Listo, probado en la placa |
| 2. DC + Hann + FFT en la placa | Listo; se valida contra numpy con `validar_nodo1.py` |
| 3. Receptor (C1–C5) en la placa | Listo; se valida contra Python con `validar_nodo1.py` |
| 4. Enlace con el nodo 3 | Listo por UART (no SPI): PCM + PARAMS + PISO; probado en la PC contra el parser del nodo 3 (`probar_enlace.py`). Falta probarlo con las dos placas |
| 5. Parámetros sin recompilar, tiempos y memoria | Comandos listos; tiempo medido en la placa (~5 ms de 32 ms); fs real del ADC 16 028 Hz |

## Archivos

| Archivo | Qué hace |
|---|---|
| `nodo1_esp32s3.ino` | Firmware: ADC1 canal 3 (GPIO4) en modo continuo por DMA; tramas de muestras cada 64 ms, análisis y trama de diagnóstico cada 32 ms. Comandos en la tabla de abajo. |
| `receptor.h` | Receptor en C++: FFT radix-2 propia (float32), piso de ruido (percentil 25 de 31 bins), criterios C1–C5 y rango de reporte de 40 dB. Traducción de `receptor_fft/src/receptor_fft.py`. |
| `trama.h` | Tramas: muestras `A5 5A` (formato de `full_system/sistema_ruido/link.py`), diagnóstico `A5 5B` y espectro `A5 5C`, todas con CRC-16/CCITT; PARAMS (`0x02`) y PISO (`0x03`) en el formato de `nodo3_esp32/communication.h`. |
| `piso_nodo3.h` | Piso de ruido para el nodo 3: el mismo estimador que `acquisition.py` (EMA λ = 0,9 + mediana de 51 bins), reutilizando la FFT del receptor. |
| `captura_nodo1.py` | PC (paso 1): recibe las muestras y corre el receptor de Python. |
| `validar_nodo1.py` | PC (pasos 2 y 3): compara la FFT de la placa con numpy y sus frecuencias con el receptor de Python, bloque a bloque; resume lo detectado y el tiempo de análisis. |
| `validacion/probar_trama.py` | Sin placa: compila `trama.h` y comprueba que sus bytes son idénticos a `link.py`. |
| `validacion/probar_receptor.py` | Sin placa: compila `receptor.h` y lo compara con Python en 7 señales (868/868 bloques idénticos). |
| `validacion/probar_enlace.py` | Sin placa: el flujo UART del nodo 1 (`enlace_host.cpp`) entra al parser real del nodo 3 en P1–P5; comprueba PCM, PARAMS y que el piso coincide con `acquisition.py` (≤ 0,02 dB). |

Las pruebas de `validacion/` necesitan `g++`; los scripts de PC necesitan numpy y
scipy, y `pyserial` para leer la placa.

## Comandos por el monitor serial

| Comando | Qué hace |
|---|---|
| `e` | Estado: tramas, análisis, desbordes, peor tiempo de análisis |
| `p` | Parámetros actuales |
| `u20` + Enter | Umbral C1 en dB sobre el piso (5 a 40) |
| `b340,1420` + Enter | Banda útil en Hz (C5) |
| `h100` + Enter | Pasa altos de adquisición en Hz (0 = apagado, por defecto 100): quita DC y graves antes del análisis, el piso que se envía y las muestras del nodo 3 |
| `r40` + Enter | Rango de reporte: no se reporta lo que esté más de N dB bajo el tono más fuerte (10 a 80) |
| `t` | Enciende/apaga el tono de prueba de 1000 Hz en GPIO5 (jumper GPIO5 → GPIO4) |
| `f` | Envía el espectro del próximo bloque (lo usa `validar_nodo1.py`) |
| `g5` + Enter | Graba 5 s (1 a 30) en la PSRAM; mientras graba no envía nada al nodo 3 |
| `x` | Reproduce lo grabado: reemplaza al micrófono en todo el sistema (nodo 3, PC) |
| `n` | Enciende/apaga el envío al nodo 3 |
| `s` | Enciende/apaga las tramas binarias por USB, para leer el monitor sin basura (el enlace al nodo 3 no cambia; `validar_nodo1.py` las necesita encendidas) |

Si el ADC recorta (códigos ≤ 8 o ≥ 4087), el nodo avisa `#saturacion ...` como máximo una vez por
segundo; `e` muestra el total. Se corrige bajando la ganancia del MAX4466 o alejando el micrófono.

### Banda útil por escenario

El nodo 1 marca como interferente todo pico validado fuera de la banda útil, y el
nodo 3 (con `diag ext`) lo suprime. Antes de cada prueba ajuste la banda:

| Escenario | Nodo 1 | Nodo 3 |
|---|---|---|
| P1 | `b900,1100` | `scen P1` |
| P2 | `b340,1420` (por defecto) | `scen P2` |
| P3 | `b150,2700` | `scen P3` |
| P4 | `b300,900` | `scen P4` |
| P5 | `b340,1420` | `scen P5` |

`validar_nodo1.py` compara contra Python con los parámetros por defecto: si se cambian,
reinicie la placa (RST) antes de validar.

## Conexión con el nodo 3

| ESP32-S3 (nodo 1) | ESP32 (nodo 3) |
|---|---|
| GPIO17 (TX de UART1) | GPIO16 (RX2) |
| GND | GND |

921600 baud, 8N1. Usa el 37 % de la UART (medido en `probar_enlace.py`).
Por cada bloque de 1024 muestras (64 ms) se envía la trama PCM y, detrás, PARAMS y
PISO calculados sobre ese mismo bloque.

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