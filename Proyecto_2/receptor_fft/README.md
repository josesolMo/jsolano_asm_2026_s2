# Taller semana 12 — Receptor y validación mediante FFT

Prototipo en Python del receptor del nodo 1: identifica las frecuencias presentes en
cada bloque con la FFT, decide si se aceptan (criterios C1–C6) y las compara con la
tabla de frecuencias del emisor. La explicación y los diagramas están en el informe
del grupo; aquí están el código, las pruebas y las fuentes de los diagramas.

## Contenido

| Ruta | Descripción |
|---|---|
| `src/receptor_fft.py` | Receptor: quitar DC, ventana Hann, FFT, piso de ruido, detección de picos, interpolación parabólica, persistencia entre bloques, clasificación, comparación con la tabla del emisor y trama con CRC-16. |
| `src/evaluar_receptor.py` | Ejecuta el receptor con los WAV de la Tarea 1 y con señales sintéticas; genera las tablas y gráficas. |
| `tests/test_receptor_fft.py` | 13 pruebas: FFT contra la DFT por definición, criterios, tabla del emisor y trama. |
| `docs/diagramas/` | Diagrama de bloques y diagrama de flujo de la validación (fuente TikZ `.tex`, más PDF y PNG). Dibujados a mano, no generados por el código. |
| `resultados/` | CSV y figuras generados por `evaluar_receptor.py`. |

## Criterios de validación

| Criterio | Regla |
|---|---|
| C1 Prominencia | Máximo local con 20 dB sobre el piso de ruido local |
| C2 Rango | Entre 46.9 Hz y fs/2 − 2 bins |
| C3 Forma tonal | Ancho a −6 dB de 3 bins o menos |
| C4 Persistencia | Presente en 3 de los últimos 4 bloques (histéresis: se mantiene con 2) |
| C5 Clasificación | Útil (dentro de la banda útil), interferente o armónico |
| C6 Tabla del emisor | Recibida si está a ±1 bin (15.6 Hz) de una frecuencia del emisor; ausente si no aparece; no esperada si está fuera de la tabla y a menos de 40 dB del tono más fuerte |

Parámetros: fs = 16 kHz, N = 1024 (Δf = 15.625 Hz), 50 % de solapamiento.
Las tablas por modo del emisor (`emisor_esp32/senales.h`, comandos 1 a 4) están en `TABLAS_POR_MODO` dentro de `src/receptor_fft.py`.

## Uso

Desde la carpeta `taller-fft-sys/` (requiere numpy, scipy y matplotlib):

    python -m receptor_fft.src.evaluar_receptor
    python -m unittest discover -s receptor_fft/tests -t . -v

Si la carpeta `tarea1_grupo/` está junto a `taller-fft-sys/`, también se evalúan sus
tres casos; si no, se omiten con un aviso.

Para recompilar los diagramas:

    cd receptor_fft/docs/diagramas
    pdflatex diagrama_bloques_receptor.tex
    pdflatex diagrama_flujo_validacion.tex

## Alcance

Es una simulación en la computadora (Python 3 con numpy, scipy y matplotlib), no
firmware. Sirve como referencia para validar la implementación del receptor en el
microcontrolador del nodo 1.