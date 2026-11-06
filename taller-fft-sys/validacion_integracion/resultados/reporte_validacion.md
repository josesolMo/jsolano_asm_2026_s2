# Validacion FFT e integracion - CE1110

**Alcance:** comprobacion digital del prototipo Python y del pipeline existente.
**Configuracion:** fs = 16000 Hz; N = 1024; resolucion = 15.625 Hz/bin; tolerancia = +/- 15.625 Hz.
**Resultado espectral:** 10/10 frecuencias dentro de tolerancia.

## Criterios de validacion

- Se seleccionan 1024 muestras, se elimina la componente continua y se aplica ventana Hann.
- Se detectan picos locales del espectro real con magnitud >=18 % del pico dominante.
- Una frecuencia cumple si uno de los picos detectados difiere, como maximo, un bin FFT.
- Se ejecuta el pipeline original del equipo para P1, P2 y P5 sin modificarlo.
- Las metricas reportadas son los valores calculados por el codigo del proyecto.

## Deteccion de componentes

| Modo | Esperada (Hz) | Detectada (Hz) | Error (Hz) | Cumple |
|---|---:|---:|---:|:---:|
| 1 - seno 1000 | 1000.000 | 1000.000 | 0.000 | Si |
| 2 - seno 2500 | 2500.000 | 2500.000 | 0.000 | Si |
| 3 - multitono | 440.000 | 437.500 | 2.500 | Si |
| 3 - multitono | 880.000 | 875.000 | 5.000 | Si |
| 3 - multitono | 1320.000 | 1312.500 | 7.500 | Si |
| 4 - transitoria | 600.000 | 593.750 | 6.250 | Si |
| 5 - melodia nota 1 | 220.000 | 218.750 | 1.250 | Si |
| 5 - melodia nota 2 | 261.630 | 265.625 | 3.995 | Si |
| 5 - melodia nota 3 | 329.630 | 328.125 | 1.505 | Si |
| 5 - melodia nota 4 | 440.000 | 437.500 | 2.500 | Si |

## Metricas de integracion

| Caso | Filtro | SNR entrada (dB) | SNR salida (dB) | Mejora (dB) | Atenuacion (dB) |
|---|---|---:|---:|---:|---:|
| P1 | FIR | 5.03 | 33.44 | 28.41 | 40.76 |
| P1 | IIR | 5.03 | 26.13 | 21.10 | 40.89 |
| P1 | ESPECTRAL | 5.03 | 43.75 | 38.73 | 38.88 |
| P2 | FIR | 0.03 | 34.10 | 34.07 | 45.01 |
| P2 | IIR | 0.03 | 14.21 | 14.18 | 44.56 |
| P2 | ESPECTRAL | 0.03 | 12.47 | 12.45 | 12.42 |
| P5 | FIR | 0.01 | 2.66 | 2.64 | 2.38 |
| P5 | IIR | 0.01 | 3.86 | 3.85 | 3.91 |
| P5 | ESPECTRAL | 0.01 | 9.93 | 9.91 | 10.14 |

## Discusion

P1 evalua la separacion entre el tono util de 1000 Hz y un interferente de 2500 Hz.
P2 examina tres tonos utiles (440, 880 y 1320 Hz) ante ruido de alta frecuencia.
P5 incluye interferencia dentro de la propia banda util; su eliminacion puede degradar la componente de 1320 Hz. No se exige mejora positiva en todos los filtros.

## Limitaciones y coherencia con el hardware

- Todas estas evidencias son simuladas; no certifican captura real por ADC ni salida analogica.
- El emisor ESP32 del equipo utiliza DAC de 8 bits (GPIO25), mientras que el prototipo de alto nivel simula cuantizacion DAC/ADC de 12 bits: falta una prueba cruzada de resolucion, ganancia y posible distorsion analogica.
- fs = 16000 Hz es nominal: se requiere osciloscopio o adquisicion externa para medir la tasa de muestreo efectiva de la placa.
- Los archivos CSV y PNG son parte de la evidencia reproducible.

## Referencias internas

- `taller-fft-sys/full_system/sistema_ruido/{config,signals,scenarios,pipeline,metrics}.py`.
- `taller-fft-sys/emisor_esp32/{senales.h,emisor_esp32.ino}`.
- Taller de sesion asincronica, semana 12, CE1110 (21-30 octubre 2026).
