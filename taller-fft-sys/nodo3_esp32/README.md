# Nodo 3 — Filtrado espectral adaptativo FFT/IFFT y evaluación (ESP32)

CE1110, Proyecto Grupal 1. Sistema distribuido de reducción de ruido en tiempo real: este nodo recibe bloques PCM,
los filtra en el dominio de la frecuencia (sustracción espectral adaptativa con rechazo de interferentes), reconstruye la
señal por solape y suma y la entrega por el DAC. También calcula las métricas de evaluación.

> ## Estado de verificación (léase primero)
> | Qué | Estado |
> |---|---|
> | Código implementado | Sí: UART, parser, FFT/IFFT, filtro, overlap‑add, DAC continuo, métricas, consola |
> | Compilación Arduino (`esp32:esp32:esp32`, núcleo 3.3.12) | **Aprobada**, sin advertencias con `--warnings all` |
> | Pruebas simuladas en el PC (C++ del firmware contra Python) | **Aprobadas** (43 comprobaciones, `tests/resultados_pc.txt`) |
> | **Pruebas en la placa** (UART real, DAC, tiempos, memoria en ejecución) | **PENDIENTES. Nada de esto se ha ejecutado en una ESP32.** |
>
> No había ninguna placa conectada al preparar este trabajo. Toda cifra de tiempo, memoria en ejecución o calidad del
> DAC que aparezca en este documento como «a medir» debe obtenerse con los comandos `bench` y `status` de §9.

## 1. Función del nodo

```
 Nodo 1 ──UART2──> [tarea RX: parser + CRC] ──cola──> [loop: diag, FFT, G, IFFT, OLA] ──anillo──> [tarea DAC] ──> GPIO25
 (ADC, FFT,           núcleo 0                           núcleo 1                         4096 muestras   núcleo 0, I2S/DMA
  diagnóstico)                                                    └─ métricas ─> USB (monitor serial)
```

Por cada bloque de 1024 muestras (64 ms) se hacen dos avances de 512 muestras (solape del 50 %):

1. historial `[anterior | nuevo]` × ventana raíz de Hann (periódica) → FFT compleja de 1024 puntos;
2. potencia por bin `P[k] = |X[k]|²/Σw²`;
3. ganancia `G[k] = √clip(1 − α·piso[k]/(P[k]+ε), β², 1)`; en ±`notch` bins de cada interferente `G = G_interf`;
4. `Y[k] = X[k]·G[k]` para `k = 0…512` y simetría conjugada `Y[N−k] = Y[k]*` explícita; IFFT;
5. ventana de síntesis y *overlap‑add*; la salida sale con un retardo de `HOP` muestras.

Con `G = 1` (modo `bypass`) la reconstrucción es exacta porque `sqrt‑Hann` al cuadrado suma 1 al 50 % de solape.

## 2. Hardware compatible

ESP32 clásica (ESP‑WROOM‑32), p. ej. DevKit de 30 pines. **No** es válido el ESP32‑S3 ni el C3 (no tienen DAC). Lógica de 3,3 V.

| Función | Pin |
|---|---|
| UART2 RX (desde Nodo 1) / TX | GPIO16 / GPIO17 |
| UART1 RX (referencia limpia, solo pruebas) | GPIO18 |
| Salida DAC1 | GPIO25 |
| USB | programación y monitor serial (115 200 baud) |

## 3. Conexiones

```
 Nodo 1 (ESP32-S3)                 Nodo 3 (ESP32 DevKit)                      Reconstrucción analógica
 TX (UART por pines) ───────────>  GPIO16 (RX2)
 GND ───────────────────────────── GND
                                   GPIO25 (DAC1) ──[ R 1 kΩ ]──┬──> osciloscopio / entrada de línea
                                                               │       (acoplar en AC; si la carga < ~10 kΩ,
                                                              [ C 22 nF ]  poner un seguidor de op‑amp)
                                                               │
                                                              GND
 (solo pruebas) fuente de referencia TX ──> GPIO18 (RX1)
```

**Filtro de reconstrucción.** El DAC mantiene cada muestra durante 1/16 kHz (retenedor de orden cero): atenúa por sinc
(−0,06 dB a 1 kHz, −0,5 dB a 3 kHz, −2,9 dB a 7 kHz) y deja réplicas en `fs ± f` (la de un tono de 1 kHz cae en 15 kHz,
−23,6 dB por el sinc). El RC de arriba (fc ≈ 7,2 kHz) añade ≈ 7 dB más en 15 kHz: son **valores de partida sin verificar en
banco**; para más rechazo use un filtro de 2.º orden. El DAC tiene **8 bits**: SNR ideal de onda llena ≈ 49,9 dB. **No conectar
parlantes directamente.**

## 4. Dependencias

| Dependencia | Versión | Nota |
|---|---|---|
| Arduino‑ESP32 (Espressif) | **3.3.12** (la del emisor) | `#warning` si es otra |
| FFT: **esp‑dsp** | incluida en el núcleo 3.3.12 | `dsps_fft2r_fc32` (rutina ensamblada `ae32`), tabla propia de 4 KB. Verificado que enlaza |
| API DAC | `driver/dac_continuous.h` (ESP‑IDF 5.x, ya en el núcleo) | `CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=y` se comprueba al compilar |
| Ninguna biblioteca externa | — | — |

**Por qué esp‑dsp y no arduinoFFT.** El núcleo 3.3.12 ya incluye `esp‑dsp` precompilada con la FFT radix‑2 de punto flotante
optimizada en ensamblador para el núcleo Xtensa LX6 y sin dependencias que instalar; `arduinoFFT` es C++ genérico. No se
comparó su velocidad (no hay placa): la elección se basa en que viene incluida y optimizada, y el tiempo real debe medirse con
`bench`. Con `-DNODO3_HOST` el mismo código usa un radix‑2 propio, para poder probar la lógica en el PC.

## 5. Instalación, compilación y carga

**Arduino IDE 2.x**
1. *Archivo → Preferencias → URLs adicionales:* `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. *Gestor de tarjetas:* instalar **esp32 by Espressif Systems 3.3.12**.
3. Abrir `taller-fft-sys/nodo3_esp32/nodo3_esp32.ino` (la carpeta debe llamarse igual que el `.ino`).
4. *Herramientas:* Placa **ESP32 Dev Module**; esquema de partición por defecto; el puerto de la placa.
5. Subir. Abrir el monitor serial a **115 200 baud** (fin de línea: *Nueva línea*) y escribir `help`.

**Línea de comandos** (`arduino-cli`; ya instalado en `~/.local/bin` en esta laptop)
```bash
cd taller-fft-sys/nodo3_esp32
./compilar.sh                 # solo compila (instala el núcleo 3.3.12 si falta)
arduino-cli board list        # identificar el puerto de la placa
./compilar.sh /dev/ttyUSB0    # compila y carga en ESE puerto (no se carga nada automáticamente)
```

Resultado de compilar (medido con `arduino-cli`, no en ejecución): programa **338 888 B (25 %)** de 1 310 720; variables
globales **123 956 B (37 %)** de 327 680. Mayores objetos estáticos: filtro 18,5 KB, diagnóstico local 14,3 KB, cola de bloques
8 KB, referencia 8 KB, tabla FFT 4 KB, anillo del DAC ~4 KB. El heap libre en ejecución es **a medir** (`status`).

## 6. Parámetros configurables (sin recompilar)

Por el monitor serial: `set alfa|beta|ginterf|lambda <v>`, `set notch <bins>`, `set band <lo> <hi>`, `set rint <lo> <hi>`,
`mode filter|bypass|raw`, `diag local|ext`, `scen P1…P5`, `fmt adc12|q15`, `set adcmid`, `baud`, `gain`, `ref on|off`,
`src uart|gen`, `gen f1 a1 f2 a2 ruido`, `status`, `params`, `metrics [reset]`, `bench`, `stream on|off`.
Valores iniciales (los de `config.py`): α = 3,0, β = 0,1, `G_interf` = 0,01, `notch` = 4 bins, λ = 0,9.

| Modo | Salida |
|---|---|
| `filter` | filtro adaptativo |
| `bypass` | FFT + IFFT + OLA con `G = 1` (comprueba la reconstrucción) |
| `raw` | entrada retardada `HOP` muestras, sin FFT: comparación A/B justa con el mismo retardo |

`src gen` genera internamente dos tonos más ruido y permite probar el DAC y el filtro **sin Nodo 1**
(`gen 1000 0.4 2500 0.2 0.05`). `tools/simular_nodo1.py` emula al Nodo 1 por UART desde la PC (con `--ref-port` envía también la referencia).

## 7. Protocolo

Ver `docs/interfaz_nodos.md` (formatos, propuesta de la trama de piso, ambigüedades y puntos a coordinar).
Resumen: PCM `A5 5A | seq u16 | n u16 = 1024 | n × int16 | CRC16(seq..datos)` idéntica a `link.py` y al `trama.h` del Nodo 1;
PARAMS tipo `0x02` de `receptor_fft.py`; **PISO tipo `0x03` propuesta** (el PARAMS no contiene el piso de ruido que exige el
filtro). CRC‑16/CCITT‑FALSE (`crc16("123456789") = 0x29B1`, verificado).

**Velocidad.** 1024 muestras × 16 bit + 8 B de cabecera/CRC = 2056 B = 20 560 bits (8N1, 10 bits/byte) cada 64 ms = 321,25 kb/s.
A 460 800 baud la utilización es 69,7 % (mínimo que da `link.disenar_enlace` con tope del 80 %); a **921 600 (defecto, igual que el
Nodo 1)** es 34,9 %, con margen para PARAMS (≈ 5 kb/s) y PISO (≈ 17 kb/s). Recepción en una tarea aparte con buffer de 8 KB del
driver y 4 bloques en cola; si la cola se llena se descarta el bloque nuevo y se cuenta.

**Integridad.** Pérdidas por `seq` (con envoltura u16), repetidos descartados, hasta 4 bloques perdidos se rellenan con ceros para
conservar la línea de tiempo; un salto mayor reinicia el solape. Contadores en `status`: `crc_err`, `perdidos`, `rellenados`,
`repetidos`, `resync`, `desbordes_cola`, `bytes_basura`.

## 8. Métricas

| Modo | Métrica | Naturaleza |
|---|---|---|
| **Con referencia** (`ref on`, enlace C) | MSE, SNR de entrada, SNR de salida, mejora de SNR, energía útil conservada | Verdaderas; fórmulas de `metrics.py`, ventana `[N, L−N)` tras `metrics reset` |
| **Sin referencia** | atenuación en `rint`, potencia conservada en la banda útil, reducción del piso | Medidas sobre los espectros antes/después de `G` |
| Sin referencia | SNR de entrada/salida *estimado* con el piso del Nodo 1 | **Estimación del modelo, no SNR verdadero.** Rotulada así |
| Siempre | µs por bloque (prom/mín/máx, bloques > 64 ms), bloques perdidos, vacíos/desbordes/saturaciones del DAC, heap, latencia | Contadores y cálculos |

No se inventan MSE ni SNR sin referencia. Las métricas se calculan **antes** del DAC (en punto flotante), no incluyen la
cuantización a 8 bits ni el filtro analógico. Latencia algorítmica = `(N + HOP)/fs` = 96 ms; a esto se suman el DMA del DAC
(64 ms) y el anillo: **a medir**. La salida CSV (`stream on`, una línea por segundo, cabecera `#N3,…`) la lee `tools/monitor_nodo3.py`,
que la muestra en una tabla o, con `--plot`, en una gráfica local y reenvía a la placa lo que se teclee.

## 9. Pruebas

Ejecutar: `PY=.venv/bin/python ./ejecutar_pruebas.sh` (requiere g++, numpy, scipy). Resultado completo guardado en `tests/resultados_pc.txt`.

| # | Prueba | Cómo | Resultado |
|---|---|---|---|
| 1 | Compilación Arduino | `arduino-cli compile` (§5) | **Aprobada**, 0 advertencias |
| 2 | Recepción UART | parser C++ contra `link.empaquetar` y `receptor_fft.empaquetar`; fragmentación de 1 a 4096 B, basura, truncadas, BER 1e‑5…5e‑4, pérdidas 0‑20 %, `seq` con envoltura | **Aprobada (PC)**: acepta exactamente las mismas tramas que `link.desempaquetar_flujo`; 0 tramas dañadas aceptadas. **En cable real: pendiente** |
| 3 | FFT/IFFT | contra `numpy.fft` | **Aprobada (PC)**: error relativo 1,4e‑7; IFFT(FFT(x)) = x con 7e‑7 |
| 4 | Reconstrucción | bypass sobre ruido de 65 536 muestras | **Aprobada (PC)**: error 1,8e‑7 (escala ±1); sin discontinuidades añadidas |
| 5 | Filtrado P1, P2, P5 | salida C++ (float32) contra `NodoEspectral` (float64) | **Aprobada (PC)**: error ≤ 2,7e‑7 con el mismo diagnóstico; ≤ 2,6e‑7 con el diagnóstico local |
| 6 | Rendimiento | `bench` en la placa | **PENDIENTE (hardware)** |
| 7 | P1–P6 | `scen`, `src gen`, `simular_nodo1.py` | P1, P2, P5 simuladas en PC; **P3, P4, P6 no ejecutadas**; todo en placa **pendiente** |

Resultados de simulación en el PC (escenarios del proyecto, 64 bloques; Python float64 frente al C++ del firmware):

| Esc. | SNR in | SNR out (Py / C++) | ΔSNR (Py / C++) | MSE (Py / C++) | E_útil (Py / C++) | SNR out con DAC de 8 bits |
|---|---|---|---|---|---|---|
| P1 | 5,03 dB | 43,75 / 43,75 | 38,73 / 38,73 | 6,12e‑6 / 6,12e‑6 | 100 / 100 % | 41,30 dB (ΔSNR 36,28) |
| P2 | 0,03 dB | 12,47 / 12,47 | 12,45 / 12,45 | 1,27e‑3 / 1,27e‑3 | 100 / 100 % | 12,45 dB |
| P5 | 0,01 dB | 9,93 / 9,93 | 9,91 / 9,91 | 1,93e‑3 / 1,93e‑3 | 93,1 / 93,1 % | 9,92 dB |

Esto valida la **lógica**, no el hardware: son señales sintéticas del propio proyecto, sin ruido de ADC real, sin filtro analógico ni
deriva de reloj. La columna del DAC de 8 bits aplica solo cuantización ideal de 8 bits a la salida del C++.

Sin referencia, el firmware *estima* para P1 un SNR de salida de 34,9 dB frente a los 43,7 dB verdaderos, y para P5 una «energía en
banda» de 73,9 % frente a 93,1 % (esa medida incluye el ruido dentro de la banda). Por eso se rotulan como estimaciones.

Piso de ruido local frente al del Nodo 1 (Python): error relativo ≤ 2,3e‑5 en los bins a menos de 60 dB del pico; en los demás, error
absoluto ≤ 1,8e‑11 del pico (‑108 dB), que es el redondeo de float32 (P1 tiene bins 118 dB bajo el pico). Los bins interferentes
coinciden en 64/64 bloques en P1, P2 y P5.

## 10. Problemas conocidos y limitaciones

- **Nada medido en placa:** tiempos de FFT/bloque, memoria dinámica, latencia, fidelidad del DAC, comportamiento de la UART a 921 600.
  Una estimación analítica (`metrics.costo_estimado`) da ≈ 1,8 MFLOP/s para FFT+IFFT, muy por debajo de la capacidad de la ESP32, pero
  es una estimación; falta además el costo del diagnóstico local (mediana de 51 bins × 513) y la consola.
- **DAC de 8 bits**, no 12 como supone `config.py` (efecto simulado arriba). El arranque del DMA puede producir un transitorio audible/visible
  (la salida pasa del reposo a 128 al arrancar el continuo): no verificado.
- **Sin reloj común:** emisor, Nodo 1 y Nodo 3 tienen cristales distintos; con el APLL la `fs` real del DAC no está medida. El anillo se vaciará o
  llenará en operación larga (se cuenta, no se corrige).
- La ganancia espectral no se suaviza entre avances (como en Python); el solape con sqrt‑Hann lo atenúa, pero puede dejar «ruido musical».
- Una sola salida DAC (GPIO25) con modos conmutables; no hay salida simultánea de entrada y salida en GPIO26.
- Enlace D (Nodo 2 → Nodo 3) y la alineación automática de salidas **no están implementados**.
- El Nodo 1 publicado hoy no envía por UART de pines ni tramas PARAMS/PISO (ver `docs/interfaz_nodos.md`).
- Discrepancias entre módulos encontradas y cómo se resolvieron: DAC 12 vs 8 bits (se usa 8); `receptor_fft` vs `acquisition` en criterios
  de interferentes y estimador de piso (el Nodo 3 usa el de `acquisition.py` por defecto, `diag ext` usa lo recibido); sin byte de tipo en PCM
  (se desambigua por `n` y CRC); escala de las muestras (selector `fmt`).

## 11. Archivos

`nodo3_esp32.ino` (tareas y consola) · `config.h` · `communication.{h,cpp}` (CRC, parser, secuencia) · `fft_processing.{h,cpp}` ·
`spectral_filter.{h,cpp}` (filtro y diagnóstico local) · `audio_output.{h,cpp}` (DAC y anillo) · `metrics.{h,cpp}` ·
`tools/` (simulador del Nodo 1, monitor) · `tests/` · `docs/` (interfaz entre nodos, descripción técnica para LaTeX).
