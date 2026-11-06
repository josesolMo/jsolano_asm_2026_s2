# Interfaz entre nodos — lo que el Nodo 3 necesita y entrega

Revisado contra `develop` (merge del Nodo 1 en PR #13 y del Nodo 2 en PR #12). **No se modificó ningún archivo de
otros nodos.** Lo marcado como *propuesta* requiere acuerdo antes de implementarse.

## 1. Estado de cada nodo en `develop`

| Nodo | Carpeta | Qué hay hoy |
|---|---|---|
| 1 | `Proyecto_2/nodo1_esp32s3/` | ESP32‑S3, ADC1 canal 3 (GPIO4), 16 kHz por DMA. Envía por el **puerto serial USB** a 921 600 baud nominal: muestras cada 64 ms (`A5 5A`), diagnóstico cada 32 ms (`A5 5B`) y espectro a petición (`A5 5C`). Hace DC, Hann, FFT y el receptor C1–C5 en la placa. fs real del ADC medida por ellos: **16 028 Hz**. **«Enlace SPI con los nodos 2 y 3: Pendiente».** |
| 2 | `Proyecto_2/nodo2_temporal/` | FIR float32 con cuatro perfiles (TONO, MULTITONO, AUDIO, TRANSITORIA). Es un diagnóstico local: no recibe SPI ni ADC; genera su propia señal y la saca por DAC (GPIO25, APLL, 16 kHz, `128 + 60·x`). Espera muestras float centradas en cero y deja el retiro de DC «para la integración». |
| 3 | `Proyecto_2/nodo3_espectral/` | Este firmware. |

## 2. Enlace entre nodos

El Nodo 1 planea **SPI** y aún no lo define (roles maestro/esclavo, modo, reloj, forma de agrupar tramas). El Nodo 3 se
implementó con **UART** (`Serial2`, GPIO16) porque hoy el Nodo 1 emite el mismo flujo de tramas como serial. Las tramas son
independientes del medio: cambiar a SPI solo exige sustituir la fuente de bytes del parser (`FrameParser::feed`).
**Decisión pendiente con el Nodo 1**; no se implementó SPI esclavo por no existir especificación.

Velocidad UART: 921 600 baud (el `BAUD` del Nodo 1), comando `baud <n>` para cambiarla. Las muestras son 2056 B cada 64 ms =
321,25 kb/s (34,9 % de 921 600; 69,7 % de 460 800, el mínimo de `link.disenar_enlace`). Con diagnósticos (≈ 68 B cada 32 ms con
8 frecuencias) y la trama de piso propuesta sube a ≈ 365 kb/s (≈ 40 % de 921 600).

## 3. Tramas (little‑endian; CRC‑16/CCITT‑FALSE desde el byte 2, sin la sincronía)

| Trama | Formato | Origen |
|---|---|---|
| `A5 5A` muestras | `seq u16 │ n u16 = 1024 │ n × int16 │ CRC` | `link.py` y `trama::empaquetar` (idénticos) |
| `A5 5B` diagnóstico | `analisis u32 │ t_us u16 │ n u8 │ 0 │ n × (f·4 u16, amp·10⁴ u16, snr·10 i16, flags u8) │ CRC`, n ≤ 8 | `trama::empaquetarDiagnostico` del Nodo 1 |
| `A5 5C` espectro | `analisis u32 │ N u16 │ N × f32 │ 513 × f32 │ CRC` (6162 B, solo si se pide `f`) | Nodo 1; el Nodo 3 la **ignora** (conviene no pedirla en la integración) |
| **`A5 5D` piso** *(propuesta)* | `analisis u32 │ decim u8 │ nk u8 │ nk × int16 centi‑dB │ CRC` | Extensión de este trabajo; usa la misma convención de sincronía |

El Nodo 3 distingue las tramas por el segundo byte de sincronía, igual que el Nodo 1. Se verificó con el `trama.h` real del
Nodo 1 (muestras, diagnóstico y espectro) que el parser reproduce exactamente lo enviado, con cualquier fragmentación,
errores de bit y pérdidas. Flags de frecuencia (`receptor.h`): `01` interferente, `02` armónico, `04` banda útil.

**Por qué se propone la trama de piso.** El filtro de `spectral.py` necesita el piso de ruido **por bin** (513 valores):
`G[k] = √clip(1 − α·piso[k]/(P[k]+ε), β², 1)`. El diagnóstico del Nodo 1 solo trae frecuencias; su `receptor.h` calcula un
piso internamente pero no lo transmite. Por eso el Nodo 3 tiene dos modos: `diag local` (defecto; estima el piso y los
interferentes con el algoritmo de `acquisition.py`, validado contra Python) y `diag ext` (usa los interferentes del `5B` y el
piso del `5D`, con respaldo local si faltan). Con 65 nudos cada 8 bins (140 B), la mejora de SNR simulada cambia a lo sumo
0,14 dB respecto al piso completo.

## 4. Formatos numéricos

| Dato | En el cable | Conversión en el Nodo 3 |
|---|---|---|
| Muestras del Nodo 1 | `int16` con la **cuenta cruda del ADC (0…4095)**, sin centrar | `fmt adc12` (defecto): `x = (m − 2048)/2048`. `fmt q15`: `x = m/32768`. Escala completa = ±1 |
| Referencia limpia (solo pruebas, UART1) | `int16` Q15 centrado | `s = m/32768` |
| Salida | código DAC de 8 bits `128 + 60·x`, saturado a 0…255 | **Misma escala que el Nodo 2** (`conversion_dac.h`), para comparar sus salidas con el mismo instrumento |

El centro del ADC del S3 no es exactamente 2048; un DC residual pasa por el filtro y desplaza la salida (`status` muestra el DC;
`set adcmid` lo compensa). Conviene acordar con el Nodo 1 quién centra la señal (el Nodo 2 también lo deja pendiente).

## 5. Sincronización y reloj

- Las muestras llevan `seq` u16; el diagnóstico lleva `analisis` u32 (uno cada 512 muestras ⇒ el bloque de muestras `s` corresponde a los
  análisis `2s` y `2s+1`). El Nodo 3 cuenta huecos, descarta repetidos, rellena hasta 4 bloques perdidos con ceros y reinicia el solape si el salto es mayor.
- **Los relojes difieren:** el ADC del Nodo 1 midió 16 028 Hz (README del Nodo 1) y el DAC del Nodo 3 se configura a 16 000 Hz (su
  frecuencia real, con APLL, no está medida). Si esas cifras se mantienen, llegan ≈ 28 muestras/s más de las que salen y el anillo
  de salida (4096 muestras, 1024 de prefill) se llena en ≈ 110 s; desde entonces se descartan muestras y se cuenta en `desbordes`.
  Para las pruebas de 4,1 s no importa; para operación continua habría que adaptar la tasa (no implementado).
- Retardo de salida: `HOP = 512` (32 ms) + bloque (64 ms) + DMA del DAC (64 ms) + anillo (≥ 64 ms por el prefill): del orden de 220–300 ms. Es un cálculo, no una medición.

## 6. Comparación entre filtros

- Con referencia limpia, el Nodo 3 calcula MSE, SNR, mejora y energía útil con las fórmulas de `metrics.py`; el Nodo 2 debe usar las mismas fórmulas y la misma
  ventana (`[N, L−N)`, `L = 65 536`). El Nodo 2 usa perfiles por prueba (P1 TONO, P2/P5 MULTITONO, P3 AUDIO, P4 TRANSITORIA); el Nodo 3 se ajusta con `scen Pn`.
- Sin referencia solo se miden atenuación en el rango de interferencia y potencia conservada en la banda útil; el SNR es una estimación (rotulada).
- Comparación de salidas analógicas: capturar ambas DAC a la vez y alinear por correlación cruzada con un rango de retardo mucho mayor que `LAG_MAX = 64` de
  `metrics.py`. El retardo del FIR del Nodo 2 es ≈ 1,3 ms y el del Nodo 3 supera los 200 ms. No automatizado.

## 7. Conexiones eléctricas del Nodo 3 (ESP32 DevKit 30 pines, 3,3 V)

| Señal | Pin | Conecta a | Obligatorio |
|---|---|---|---|
| UART2 RX | GPIO16 | TX del Nodo 1 (hoy el Nodo 1 no tiene salida por pines; se usaría su UART o el medio que se acuerde) | sí |
| UART2 TX | GPIO17 | RX del Nodo 1 (sin uso) | no |
| GND | GND | GND común | sí |
| UART1 RX | GPIO18 | TX de la fuente de referencia (solo `ref on`) | no |
| DAC1 | GPIO25 | red RC de reconstrucción → instrumento | sí |
| USB | — | PC: programación y monitor (115 200 baud) | sí |

Nunca 5 V en un pin. GPIO16/17 están libres en ESP‑WROOM‑32, no en WROVER/PSRAM. No conectar parlantes al DAC. Si se usa SPI, estos pines cambian.

## 8. Puntos que requieren coordinación

**Con el Nodo 1**
1. Medio de enlace (SPI previsto): roles, modo, reloj, y si las tramas siguen igual. Mientras tanto el Nodo 3 usa UART.
2. ¿Aceptan la trama de piso `A5 5D`, o prefieren otro mecanismo, o el Nodo 3 sigue estimando el piso por su cuenta?
3. Estimador de piso distinto: `acquisition.py` (mediana de 51 bins sobre EMA λ = 0,9) frente a `receptor.h` (percentil 25 de 31 bins sobre un bloque). Si envían el suyo, el filtro quedará calibrado con otro estimador.
4. Centrado del ADC (quién resta el DC y con qué valor).
5. No pedir el espectro (`f`) durante la integración: ocupa 6162 B y el Nodo 3 lo descarta.

**Con Aldo (Nodo 2)**
6. Su nodo aún no recibe datos (diagnóstico local): ¿qué formato y medio usará? ¿Las mismas tramas?
7. Ventana y fórmulas de métricas idénticas; cómo capturar y alinear las dos salidas.
