# Interfaz entre nodos — lo que el Nodo 3 necesita y entrega

Documento para coordinar con el responsable del Nodo 1 y con Aldo (Nodo 2). **No se modificó ningún
archivo de otros nodos.** Todo lo marcado como *propuesta* requiere acuerdo antes de implementarse.

## 1. Estado real de cada nodo en el repositorio (revisado el 2026-11-04)

| Nodo | Dónde está | Qué hace hoy |
|---|---|---|
| Nodo 1 (referencia) | `develop`: `taller-fft-sys/receptor_fft/` | Solo **Python**: `receptor_fft.py` define la trama de parámetros (tipo `0x02`). No es firmware. |
| Nodo 1 (firmware) | rama `feature/nodo-1`, commit `c505263`, carpeta `Proyecto_2/nodo1_esp32s3/` (**no está en `develop`**) | ESP32‑S3, ADC1 canal 3 (GPIO4), 12 bits, 16 kHz por DMA. Envía **solo tramas PCM** por el **puerto serial USB** a la PC, a 921 600 baud nominal. Sin FFT, sin tramas de parámetros, sin UART de pines hacia otros nodos. |
| Nodo 2 (Aldo) | rama `feature/nodo-2` | Apunta al **mismo commit** que `feature/nodo-1`: no hay trabajo propio publicado. No se conoce su interfaz. |
| Nodo 3 | `feature/nodo3-fft-esp32`: `taller-fft-sys/nodo3_esp32/` | Este firmware. |

## 2. Enlaces

```
                      ┌────────────── Enlace A: PCM + PARAMS (+ PISO) ──────────────┐
 Emisor ──DAC──> Nodo 1 (ESP32‑S3, ADC)                                              ▼
 (ESP32)         │                                                        Nodo 3 (ESP32 clásica)
                 └── Enlace B: PCM ──> Nodo 2 (Aldo, FIR/IIR) ──DAC──┐     UART2 RX = GPIO16
                                                                     │     DAC1 = GPIO25 ──> filtro RC ──> instrumento
          Enlace C (solo pruebas): referencia limpia ──> Nodo 3 UART1 RX = GPIO18
```

| Enlace | Origen → destino | Contenido | Estado |
|---|---|---|---|
| A | Nodo 1 → Nodo 3, UART2 (GPIO16), 8N1, 3,3 V | Tramas PCM (obligatorio). PARAMS y PISO (opcionales, ver §4). | PCM: formato igual al publicado. **El Nodo 1 hoy no tiene salida UART hacia el Nodo 3**: debe agregarla. |
| B | Nodo 1 → Nodo 2 | Se asume el mismo PCM. | **Sin confirmar con Aldo.** |
| C | Fuente de referencia → Nodo 3, UART1 (GPIO18) | Señal limpia, misma trama PCM y misma numeración. Solo para medir MSE/SNR verdaderos. | Implementado en el Nodo 3; lo emula `tools/simular_nodo1.py`. |
| D | Nodo 2 → Nodo 3 (comparación FIR/IIR vs espectral) | — | **No implementado.** Hoy la comparación es externa (§6). |

Velocidad: **921 600 baud** por defecto (igual que la constante `BAUD` del firmware del Nodo 1). Una trama PCM
son 2056 B = 20 560 bits cada 64 ms = 321,25 kb/s: 34,9 % de 921 600 y 69,7 % de 460 800 (el mínimo que
calcula `link.disenar_enlace`). Con PARAMS y PISO por bloque se suman ≈ 22 kb/s (§4). Se cambia en
ejecución con `baud <n>` y debe coincidir en ambos extremos.

## 3. Formatos numéricos

| Dato | Formato en el cable | Conversión en el Nodo 3 |
|---|---|---|
| Muestras PCM del Nodo 1 | `int16` little‑endian. **Hoy: cuenta cruda del ADC, 0…4095, sin signo, sin centrar.** | `fmt adc12` (defecto): `x = (m − adc_mid)/2048`, `adc_mid = 2048`. `fmt q15`: `x = m/32768`. Escala completa = ±1,0. |
| Referencia limpia (enlace C) | `int16` Q15 centrado, siempre | `s = m/32768` |
| Salida al DAC | código de 8 bits, `128 + x·127`, saturado a 0…255 | El DAC de la ESP32 clásica es de **8 bits** (`config.py` supone 12). |

El ADC del ESP32‑S3 no está centrado en 2048 con exactitud (sesgo y offset analógico). Un DC en `x` pasa por el
filtro y desplaza la salida: el comando `status` muestra `[ENTRADA] DC`; se corrige con `set adcmid <cuentas>`
o, mejor, que el Nodo 1 centre las muestras antes de enviar (*propuesta*).

## 4. Tramas (todas little‑endian, CRC‑16/CCITT‑FALSE, init 0xFFFF)

| Trama | Formato | Origen |
|---|---|---|
| PCM | `A5 5A │ seq u16 │ n u16 = 1024 │ n × int16 │ CRC16` — el CRC cubre `seq,n,datos` (sin SYNC) | `link.empaquetar`, `trama.h` del Nodo 1 (idénticos) |
| PARAMS (`0x02`) | `A5 5A │ 02 │ sec u8 │ n u8 │ n × (f·4 u16, snr i8, flags u8) │ CRC16` — el CRC cubre desde `A5` | `receptor_fft.empaquetar`; n ≤ 8; flags: `01` interferente, `02` armónico, `04` banda útil |
| **PISO (`0x03`)** *propuesta* | `A5 5A │ 03 │ sec u8 │ decim u8 │ nk u8 │ nk × int16 (centi‑dB) │ CRC16` — el CRC cubre desde `A5` | Extensión de este trabajo |

**Por qué hace falta PISO.** El filtro de `spectral.py` calcula `G[k] = √clip(1 − α·piso[k]/(P[k]+ε), β², 1)` con el piso de
ruido **por bin** (513 valores) del Nodo 1. La trama PARAMS solo trae *frecuencias* de picos validados: permite
rechazar interferentes tonales, pero **no contiene el piso**, así que no basta para la sustracción espectral.
Opciones: (a) el Nodo 3 estima el piso por su cuenta (`diag local`, implementado y validado: replica
`NodoAdquisicion` bit a bit dentro del error de float32); (b) el Nodo 1 envía PISO (`diag ext`).

PISO transmite el piso en dB sobre `nk` nudos cada `decim` bins (propuesta: `decim = 8`, `nk = 65`, 138 B por trama,
17,3 kb/s si se envía una por bloque) y el Nodo 3 interpola linealmente en dB. Valor del nudo `j` =
`round(100·10·log10(piso[j·decim]))`, con el piso en la normalización de `acquisition.py` (`|X|²/Σw²`, Hann, escala ±1,0).
Validado en el PC: con 65 nudos la mejora de SNR cambia a lo sumo 0,14 dB respecto al piso completo (P1, P2, P5).

**Ambigüedad a resolver.** La trama PCM no tiene byte de tipo y PARAMS/PISO usan el byte siguiente a `A5 5A` como tipo,
que en una trama PCM es el byte bajo de `seq`. El parser del Nodo 3 los distingue porque una PCM siempre trae `n = 1024`
(`00 04`) en los bytes 4–5 y porque el CRC debe cuadrar (probado, incluso con `seq` bajo = `02`/`03` y con PARAMS vacío
cuyo CRC bajo es `04`). Es robusto pero frágil; la alternativa limpia es enviar PARAMS/PISO por un UART distinto o reservar
un byte de tipo en las PCM. **Decisión pendiente con el Nodo 1.**

Cadencia supuesta: una PARAMS y una PISO por cada PCM, con la misma `sec` (mod 256) que el bloque. Si el Nodo 1 usa
el avance de 512 muestras (32 ms) de `receptor_fft`, el Nodo 3 usa la última recibida (se descarta si tiene más de 5 bloques).

## 5. Identificación de bloques y sincronización

- Cada bloque PCM lleva `seq` (u16, con envoltura). El Nodo 3 cuenta huecos, ignora repetidos/antiguos y rellena hasta
  4 bloques perdidos con ceros para no desplazar la línea de tiempo; un salto mayor reinicia la historia del solape.
- Los nodos **no comparten reloj**: cada ESP32 usa su cristal (el emisor y el DAC usan APLL; la frecuencia real no está
  medida). Una diferencia de reloj de 20–50 ppm equivale a ~0,3–0,8 muestras por segundo a 16 kHz (una muestra cada pocos segundos). Para pruebas de 4,1 s es tolerable; en
  operación continua el anillo de salida (4096 muestras) acabará vaciándose o llenándose y el Nodo 3 lo contabiliza
  (`vacios`, `desbordes`) pero no lo corrige.
- Retardo de la salida del Nodo 3: `HOP = 512` muestras (32 ms) del solape + captura del bloque (64 ms) + DMA del DAC
  (64 ms) + ocupación del anillo (≥ 64 ms por el prefill) ⇒ del orden de 220–300 ms desde el primer instante de la entrada
  hasta el DAC. Es un cálculo, **no una medición**. El Nodo 2 tendrá otro retardo (FIR: (N + M/2)/fs según `metrics.py`).

## 6. Comparación entre filtros

- **Con referencia limpia (enlace C):** el Nodo 3 calcula MSE, SNR de entrada/salida, mejora y energía útil de *su* salida
  (misma fórmula que `metrics.py`, validada). Para comparar con el Nodo 2 hace falta la misma referencia en la
  misma ventana de medición; Aldo debe calcular sus métricas con las mismas fórmulas y la misma ventana
  (`[N, L−N)` con `L = 65 536`).
- **Sin referencia:** solo se pueden medir la atenuación en el rango de interferencia y la potencia conservada en la banda
  útil, y *estimar* el SNR con el piso del Nodo 1 (rotulado como estimación; sobre P1 estima 34,9 dB frente a 43,7 dB verdadero).
- **Comparación de las salidas analógicas:** capturar ambas salidas DAC simultáneamente (osciloscopio de 2 canales o
  tarjeta de audio) y alinear por correlación cruzada con un rango de retardo mucho mayor que `LAG_MAX = 64` de `metrics.py`
  (el retardo del Nodo 3 supera las 1000 muestras). Pendiente de acordar con Aldo; no está automatizado.

## 7. Conexiones eléctricas del Nodo 3 (ESP32 DevKit 30 pines, lógica 3,3 V)

| Señal | Pin ESP32 | Conecta a | Obligatorio |
|---|---|---|---|
| UART2 RX | GPIO16 | TX (UART, pin) del Nodo 1 | sí |
| UART2 TX | GPIO17 | RX del Nodo 1 (sin uso hoy) | no |
| GND | GND | GND común de todos los nodos | sí |
| UART1 RX | GPIO18 | TX de la fuente de referencia (solo `ref on`) | no |
| DAC1 | GPIO25 | red de reconstrucción RC (§README) → instrumento | sí |
| USB | — | PC (programación y monitor, 115 200 baud) | sí |

No conectar 5 V a ningún pin. GPIO16/17 están libres en módulos ESP‑WROOM‑32 pero **no** en WROVER/PSRAM: confirmar el
módulo. No conectar parlantes al DAC.

## 8. Puntos que requieren coordinación

**Con el Nodo 1**
1. Salida UART por pines hacia el Nodo 3 (hoy solo USB‑CDC a la PC) y velocidad (se asume 921 600).
2. Formato de las muestras: cuenta cruda (hoy) o Q15 centrado; en el primer caso, el valor de centro real del ADC.
3. ¿Enviará PARAMS (`0x02`) y/o PISO (`0x03`)? ¿Por el mismo UART (ambigüedad §4) o por otro?
4. Discrepancia de criterios de interferentes: `receptor_fft.py` (umbral 20 dB, persistencia 3/4, hasta 8 frecuencias, sin
   límite de rango dinámico) frente a `acquisition.py` (15 dB, dentro de 40 dB del mayor, zumbido de red). En P1 el primero
   reporta ≈ 7,9 interferentes por bloque (el tono más productos de cuantización del ADC) y el segundo 1,0. Probado: el efecto
   sobre la mejora de SNR es ≤ 0,14 dB, pero conviene unificar.
5. Estimador de piso: `acquisition.py` usa mediana de 51 bins sobre un promedio EMA (λ = 0,9); `receptor_fft.py` usa percentil 25 de
   31 bins sobre un solo bloque. Debe acordarse cuál es el que se transmite en PISO.

**Con Aldo (Nodo 2)**
6. Formato y fuente de entrada del Nodo 2 (¿mismas tramas PCM?, ¿mismo `seq`?).
7. Ventana y fórmulas de métricas idénticas; cómo se captura y alinea la salida de ambos filtros.
8. Si se quiere comparar dentro del Nodo 3, que el Nodo 2 envíe su salida como PCM por el enlace D (no implementado).
