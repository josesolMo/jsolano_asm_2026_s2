# Nodo 1 — Adquisición y análisis (ESP32-S3)

Paso 1: captura continua del ADC a 16 kHz y envío de bloques a la PC, donde el
receptor de Python (`receptor_fft`) valida las frecuencias. Todavía no hay FFT en el
microcontrolador.

## Archivos

| Archivo | Qué hace |
|---|---|
| `nodo1_esp32s3.ino` | Firmware: ADC1 canal 3 (GPIO4) en modo continuo por DMA a 16 kHz; arma tramas de 1024 muestras y las envía por el puerto serial USB. Responde `e` con el estado (tramas enviadas, desbordes). |
| `trama.h` | Formato de trama, idéntico a `full_system/sistema_ruido/link.py`: `[A5 5A][secuencia u16][n u16][n × int16][CRC-16]`. Se usa en la placa y en la prueba de escritorio. |
| `captura_nodo1.py` | PC: lee el puerto, valida tramas (CRC y secuencia), revisa nivel y recortes del ADC, y corre el receptor contra la tabla del modo del emisor. Guarda `.bin`, `.npy` y `.wav` en `capturas/`. |
| `validacion/probar_trama.py` | Compila `trama.h` en la PC y comprueba que sus bytes son idénticos a los de `link.py` y que el CRC detecta errores. No necesita placa. |

## Compilar y cargar (Arduino IDE)

- Paquete **Arduino-ESP32 3.3.12** (el mismo del emisor).
- Placa: **ESP32S3 Dev Module**.
- Si conecta por el puerto USB nativo de la placa: *USB CDC On Boot: Enabled*. Si usa el puerto con convertidor USB-serial (a veces marcado *COM* o *UART*): *Disabled*.

## Conexión de prueba (con el emisor)

```
ESP32 clásico GPIO25 (DAC1) ──→ ESP32-S3 GPIO4 (ADC1 canal 3)
ESP32 clásico GND ───────────→ ESP32-S3 GND
```

## Uso

Desde `taller-fft-sys/`:

    pip install pyserial
    python nodo1_esp32s3/validacion/probar_trama.py
    python nodo1_esp32s3/captura_nodo1.py --puerto COM5 --segundos 5 --modo 3

`--modo` es el modo que se está reproduciendo en el emisor (1 = 1000 Hz, 2 = 2500 Hz,
3 = 440/880/1320 Hz, 4 = transitoria de 600 Hz). Para analizar de nuevo una
captura guardada: `--archivo nodo1_esp32s3/capturas/captura_<fecha>.bin`.
