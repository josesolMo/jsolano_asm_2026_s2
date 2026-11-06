# Proyecto 2 — Reducción de ruido distribuida

El sonido contaminado entra por el micrófono del **nodo 1**, que lo digitaliza, lo
analiza con la FFT y envía al **nodo 3** las muestras, las frecuencias interferentes
validadas y el piso de ruido. El nodo 3 filtra y reproduce el sonido limpio en un
parlante. Desde la consola del nodo 3 se elige qué se escucha: la entrada sin
filtrar, el filtro espectral (nodo 3) o el filtro temporal FIR (método del **nodo 2**).
Todo el procesamiento ocurre en los microcontroladores; la PC solo envía comandos y
grafica las métricas.

```
MAX4466 ─> GPIO4 ┌──────────── Nodo 1: ESP32-S3 ────────────┐ GPIO17 ─UART 921600─> GPIO16 ┌──────── Nodo 3: ESP32 ────────┐ GPIO25 ─> RC ─> LM386 ─> parlante
                 │ ADC 16 kHz · FFT 1024 · receptor C1–C6   │   PCM + PARAMS + PISO          │ espectral FFT/IFFT  |  FIR     │  (DAC 8 bits)
                 │ grabar / reproducir (PSRAM)              │                                │ métricas · consola             │
                 └── USB: diagnóstico para la PC ───────────┘                                └── USB: comandos y métricas ────┘
```

## Carpetas

| Carpeta | Qué contiene |
|---|---|
| `nodo1_esp32s3/` | Adquisición, FFT, receptor y enlace UART (ESP32-S3). Ver su `README.md`. |
| `nodo2_temporal/` | Diseño y validación de los filtros FIR (perfiles TONO, MULTITONO, AUDIO, TRANSITORIA). |
| `nodo3_esp32/` | Filtro espectral adaptativo, modo `fir` con los coeficientes del nodo 2, salida DAC y métricas (ESP32 clásico). |
| `receptor_fft/` | Receptor de referencia en Python (criterios C1–C6) contra el que se valida el nodo 1. |
| `full_system/` | Simulación completa en Python: escenarios P1–P6, filtros de referencia y métricas. |

## Conexiones

| Desde | Hacia | Nota |
|---|---|---|
| MAX4466 OUT | Nodo 1 GPIO4 | VCC del micrófono a 3V3 del S3, GND común |
| Nodo 1 GPIO17 (TX UART1) | Nodo 3 GPIO16 (RX2) | 921600 baud, 8N1 |
| Nodo 1 GND | Nodo 3 GND | obligatorio |
| Nodo 3 GPIO25 (DAC1) | 1 kΩ en serie + 33 nF a GND | filtro de reconstrucción (corte ≈ 4,8 kHz) |
| Salida del RC | 1 µF ─> potenciómetro ─> entrada + del LM386 | parlante de 8 Ω 2 W en la salida del LM386 |
| Nodo 3 GPIO26 (DAC2) | otro RC igual (1 kΩ + 10 nF) | salida del nodo 2 (FIR), siempre activa: osciloscopio o segundo parlante |
| MAX4466 OUT | RC anti-aliasing (p. ej. 2,2 kΩ + 10 nF, corte ≈ 7 kHz) ─> GPIO4 | fs = 16 kHz: lo que pase de 8 kHz se dobla |

La placa 2 implementa los nodos 2 y 3 (el enunciado lo permite en un micro multinúcleo, con dos
rutas independientes y dos salidas comparables): las dos rutas procesan cada bloque a la vez y
cada una tiene su DAC. GPIO25 (parlante) saca lo que se elige con `mode`; GPIO26 saca siempre el FIR.

Prueba sin micrófono: un jumper GPIO5 → GPIO4 en el nodo 1 y el comando `t` inyectan un
tono de 1000 Hz generado por la propia placa.

## Placas (Arduino IDE, Arduino-ESP32 3.3.12)

| Nodo | Placa | Ajustes |
|---|---|---|
| 1 | ESP32S3 Dev Module | Flash 16MB, PSRAM: OPI PSRAM, USB CDC On Boot: Disabled, puerto UART |
| 3 | ESP32 Dev Module | valores por defecto |

Los tres firmwares se compilaron con Arduino-ESP32 3.3.12 sin advertencias.
Memoria estática: nodo 1 20 %, nodo 3 37 %. El nodo 3 reserva unos 30 KB de heap
al arrancar, para el modo `fir`.

## Revisión: cómo correr la demo

1. **Nodo 3**: abrir su monitor (115200 baud), o mejor
   `python nodo3_esp32/tools/monitor_nodo3.py COMx --plot`, que reenvía lo que se
   escribe y grafica las métricas. Escribir `scen P1` (o el escenario de la prueba) y
   `stream on`. El nodo 3 usa por defecto el diagnóstico del nodo 1 (`diag ext`).
   Si el nodo 1 muestra `#saturacion`, bajar la ganancia del MAX4466 o alejar el micrófono.
2. **Nodo 1**: en su monitor, fijar la banda útil del escenario (`b900,1100` para P1;
   la tabla está en `nodo1_esp32s3/README.md`). Con `e` se ve el estado del enlace.
3. **En vivo**: el profe reproduce el sonido contaminado frente al micrófono.
   `recomendar` en el nodo 3 dice qué método conviene para ese ruido y por qué
   (interferentes respecto al FIR, cuánto ruido cae en la banda útil). Luego se
   conmuta qué se escucha:
   - `mode raw`: la entrada sin filtrar;
   - `mode filter`: el filtro espectral;
   - `mode fir`: el filtro temporal del nodo 2.
4. **Después, sin interferencia con el sonido del profe**: en el nodo 1, `g10` graba
   10 s mientras suena el contaminado; cuando termine, `x` lo reproduce por todo el
   sistema y el nodo 3 lo saca ya limpio. Se puede repetir `x` con otro `mode`.
5. Gráficas: cerrar el monitor del nodo 3 y correr
   `python nodo3_esp32/tools/graficar_captura.py COM7`: formas de onda y espectros de la
   entrada y de las dos salidas, con la atenuación y la energía conservada de cada método.
6. Métricas: `metrics` en el nodo 3 muestra los dos métodos lado a lado (las métricas
   sin referencia son estimaciones y están rotuladas así). El micrófono y el parlante
   deben quedar separados para evitar realimentación acústica; con `g` / `x` el sonido
   limpio sale después y no se mezcla con el contaminado.

## Pruebas en la PC (sin placas)

Desde esta carpeta, con g++, numpy y scipy:

    bash probar_todo.sh

Corre el receptor, las tramas, el enlace nodo 1 → nodo 3, el DSP del nodo 3 contra
Python, el modo `fir` y la verificación de los FIR del nodo 2.