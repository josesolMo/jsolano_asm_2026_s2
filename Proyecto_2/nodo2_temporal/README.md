# Filtro temporal del nodo 2

Implementacion independiente de SPI, con diagnostico local de DAC. Reutiliza las especificaciones
del prototipo `full_system/sistema_ruido/temporal_filters.py`: muestreo de 16000 Hz,
transicion de 700 Hz, rizado maximo de 0.5 dB y rechazo minimo de 40 dB.
El firmware del nodo 1 y el prototipo existente no se modifican.

## Seleccion

Se selecciona un FIR Parks-McClellan de fase lineal, tipo I, con coeficientes y
acumulacion float32. El IIR eliptico candidato requiere orden 4 o 5 y menos
operaciones; sus polos quedan dentro del circulo unidad en el diseno de referencia.
El FIR facilita preservar la fase relativa, alinear la referencia con un retardo
entero y analizar transitorios, con un costo estimado de 656000 a 688000 MAC/s.
Este costo es una estimacion, pendiente de medir en el microcontrolador.

No existe un unico pasa bajas que rechace 2500 Hz en P1 y conserve simultaneamente
los armonicos de audio hasta 2640 Hz en P3. Se ofrecen perfiles explicitos:

| Perfil | Pruebas | Paso desde DC hasta | Rechazo desde | Orden | Retardo de grupo |
|---|---|---:|---:|---:|---:|
| TONO | P1 | 1100 Hz | 1800 Hz | 40 | 1.25 ms |
| MULTITONO | P2 y P5 | 1420 Hz | 2120 Hz | 42 | 1.3125 ms |
| AUDIO | P3 | 2700 Hz | 3400 Hz | 40 | 1.25 ms |
| TRANSITORIA | P4 | 900 Hz | 1600 Hz | 42 | 1.3125 ms |

MULTITONO conserva 440, 880 y 1320 Hz; TONO conserva 1000 Hz; AUDIO conserva las
cuatro notas de la melodia del prototipo y sus seis armonicos; TRANSITORIA conserva
el portador de 600 Hz, pero modifica la envolvente y las componentes fuera de su
banda. El retardo indicado es del filtro: capturar 1024 muestras agrega 64 ms antes
del procesamiento por bloque, ademas del transporte, calculo y salida futuros.

Se conserva el enfoque pasa bajas del prototipo. No elimina zumbido de 60 Hz,
ruido dentro de banda ni impulsos completos. En P5 queda ruido superpuesto a los
tonos utiles; no se promete separacion perfecta. AUDIO tambien deja pasar 2500 Hz.
P6 requiere seleccionar una banda compatible con la senal desconocida; estos
cuatro perfiles no garantizan conservar cualquier audio arbitrario.

## Uso de C++

Incluir `filtro_fir.h` y `coeficientes.h`, y mantener una instancia de
`temporal::FiltroFIR<temporal::MULTITONO.size()>` inicializada con
`temporal::MULTITONO`. Llamar a `procesar(entrada, salida, cantidad)` en cada
bloque, con muestras float centradas en cero y la frecuencia de muestreo indicada.
La longitud puede variar; se admite usar el mismo arreglo para entrada y salida.
Los arreglos deben tener al menos `cantidad` elementos y no solaparse parcialmente.
Una llamada de longitud cero no cambia el estado.

El objeto conserva su historial. Usar `reiniciar()` solo para iniciar una captura
independiente; no entre bloques consecutivos. Cambiar de perfil exige otra instancia
y reiniciar introduce un transitorio. La seleccion durante operacion y la transicion
entre perfiles quedan para la integracion posterior.

No se recorta ni normaliza cada bloque. Una entrada con pico 1 puede producir salida
mayor que 1: `diseno.json` incluye la cota sum(abs(b)). El retiro de DC del ADC
queda para la integracion de adquisicion. La conversion DAC ya incluye margen y saturacion.
La clase no usa Arduino, memoria dinamica ni dependencias externas.

## Reproduccion y validacion

Desde la raiz del repositorio, con Python, las dependencias de `requirements.txt`
y g++ o clang++ en PATH:

```sh
# Regenerar los perfiles y el informe desde las especificaciones del prototipo.
python Proyecto_2/nodo2_temporal/validacion/disenar.py
# Compilar la misma clase C++ que se integrara y compararla contra SciPy.
python Proyecto_2/nodo2_temporal/validacion/verificar.py
```

`disenar.py` genera `coeficientes.h` y `diseno.json`; `verificar.py` genera
`validacion/resultados.json`. Los ejecutables y muestras temporales se eliminan.
No se importa el prototipo porque su configuracion crea directorios al importarse.

La referencia es `scipy.signal.lfilter` en float64 con los mismos coeficientes
float32 exportados a C++. Se exigen atol=2e-6 y rtol=2e-5 para entradas de pico
menor o igual que 1. Se prueban ruido, escalon, impulso en muestra 1023, multitono,
mezcla de 1000/2500 Hz, armonico de audio de 2640 Hz y transitoria de 600 Hz.
Cada perfil se verifica con bloques de 1, 257, 1024 y longitud completa, tanto con
salida separada como en el mismo arreglo: 224 comparaciones en total. Tambien se
comprueban reinicio, llamada vacia, simetria de coeficientes y especificaciones en
una malla densa que incluye los bordes y Nyquist. Las particiones producen salidas
identicas bit a bit. Son pruebas de escritorio, no mediciones del hardware.

## Diagnostico en Arduino IDE

Abrir `nodo2_temporal.ino`, seleccionar la placa y el puerto y cargarlo. En el
monitor serial a 115200 baudios, enviar `t` para la prueba del impulso o `m` para
medir los cuatro perfiles. `?` muestra los comandos. No se necesitan conexiones
externas para estas pruebas.

La medicion usa `micros()` alrededor de `procesar()`, con interrupciones habilitadas,
10 bloques de calentamiento y 200 bloques medidos por perfil. Informa minimo,
promedio y maximo en microsegundos, porcentaje del presupuesto de 64000 us y
cantidad de bloques que alcanzan o exceden ese presupuesto. La salida se consume
fuera del intervalo cronometrado para evitar eliminar el calculo por optimizacion.
La generacion de entrada, verificacion, salida serial y `delay(1)` quedan fuera.
El historial se mantiene entre iteraciones.

Para evaluar el resultado, guardar el reporte completo junto con el modelo de placa,
version de Arduino-ESP32 y frecuencia de CPU seleccionada en el IDE. Conviene repetir
`m` tres veces y revisar el mayor maximo observado. Un maximo inferior a 64000 us
indica margen del filtro aislado durante esas mediciones, no una garantia de tiempo
real del sistema: falta incluir recepcion, conversion, DAC y sus buffers. Este
diagnostico tampoco reproduce muestras a 16 kHz ni mide latencia de extremo a extremo.
El maximo observado no es una cota formal del peor caso.

## Salida DAC local por GPIO25

Requiere ESP32 clasico (ESP32 Dev Module) y Arduino-ESP32 3.3.12, con alineacion
automatica DMA de 16 bits habilitada en el paquete. No es compatible con el
ESP32-S3 del nodo 1, que no tiene este DAC. Se usa DAC_CHAN_0 (GPIO25), APLL y
16000 muestras/s nominales. La tasa real requiere medicion externa.

`conversion_dac.h` convierte muestras centradas en cero mediante
codigo = redondear(128 + 60 * muestra), con saturacion a [0, 255]. La escala fija
evita normalizacion por bloques. Para entradas al FIR de pico <= 1 y estado inicial
cero, la mayor cota de salida entre los perfiles es 1.865; 60 * 1.865 < 127 deja
margen a ambos lados del centro. Se cuentan las muestras saturadas antes de
redondear. NaN o infinito producen codigo 128 y aumentan un contador independiente.
Los contadores de conversion se reinician al iniciar una reproduccion; los errores
API se acumulan desde el arranque.

| Comando | Accion |
|---|---|
| `i` | Prepara y reproduce el diagnostico filtrado desde cero |
| `p` | Cancela DMA, libera el canal continuo y fija el codigo 128 |
| `e` | Muestra progreso, recortes, entradas no finitas y errores DMA/API |
| `t` o `m` | Detiene DAC antes de ejecutar las pruebas previas |

El diagnostico genera 65536 muestras (4.096 s) de 1000 Hz a amplitud 0.6 y
2500 Hz a amplitud 0.3, con rampas de 20 ms. Pasa por el FIR MULTITONO, que conserva
1000 Hz y rechaza 2500 Hz. Agrega 1024 muestras para vaciar el historial y terminar
centrado: 66560 muestras, 4.160 s nominales en total. El arreglo uint8_t ocupa
66560 bytes, ademas de 16384 bytes de buffers DMA y los recursos del driver.

`salida_dac.h` reutiliza el ciclo de vida del DAC del emisor del taller. Los bloques
preparados se cargan a DMA; el hardware temporiza las conversiones.
Las escrituras al driver son de hasta 8192 muestras (512 ms nominales), para
alimentar varios descriptores por llamada. El FIR sigue procesando bloques de
1024 muestras. La ultima escritura se ajusta a las muestras pendientes. Las
escrituras pueden bloquear mientras se libera espacio DMA; los comandos se
atienden entre llamadas, por lo que este diagnostico no garantiza parada inmediata.
La ISR registra longitudes notificadas por los eventos y finales. El conteo de
eventos se conserva como diagnostico: no se usa como medicion fisica ni como
condicion unica de error. El driver devuelve descriptores a su cola antes de leer
la longitud que entrega al callback, por lo que su reutilizacion puede afectar
esa suma. La tarea exige toda la captura cargada y un unico fin de cadena;
un cierre previo o multiples cierres registran discontinuidad. El campo
`Completadas al cierre de cadena` expresa la confirmacion de la cadena cargada
por `on_stop`, no una medicion independiente. Las escrituras parciales y fallos API tambien
se reportan. El reposo se confirma por API, no por medicion de tension.

Esta etapa reproduce una captura preparada localmente: no implementa todavia
recepcion SPI ni filtrado concurrente con la reproduccion. No demuestra tiempo real
de extremo a extremo. Al enviar `e` tras la finalizacion se esperan 66560 muestras
cargadas y completadas al cierre, cero recortes, cero entradas no finitas y cero
agotamientos. El conteo bruto de eventos puede diferir y se muestra por separado.
Un error de continuidad no se presenta como fallo de una API; anteriormente el
diagnostico generaba artificialmente ESP_ERR_INVALID_STATE al diferir la suma.

Para inspeccion fisica usar GPIO25 y GND con osciloscopio o entrada de alta
impedancia. El codigo 128 corresponde aproximadamente a media escala. La conexion
a audio requiere filtro analogico de reconstruccion, acoplamiento que retire DC
y una entrada amplificada adecuada; no conectar un parlante directamente al GPIO.
El filtro analogico y su caracterizacion quedan pendientes.

`validacion/test_conversion_dac.cpp` verifica escala, extremos, entradas no finitas,
acumulacion de recortes y la cota de margen de los cuatro perfiles. Se compila con
g++ o clang++ en escritorio. La integracion tambien se comprobo en sintaxis con
el compilador Xtensa y encabezados oficiales de Arduino-ESP32 3.3.12; falta la
compilacion completa desde Arduino IDE y la verificacion fisica en placa.

## Modelo y recursos

Para cada perfil, los coeficientes de `diseno.json` definen
H(z) = sum desde k=0 hasta M de b[k] z^(-k), con M igual al orden.
La ecuacion aplicada es y[n] = sum b[k] x[n-k]. Al expresarlo como funcion racional
con denominador z^M, sus polos estan en el origen y la ROC causal es |z| > 0.
La respuesta impulsional finita asegura estabilidad BIBO; la simetria determina
el retardo M/2 muestras. La obtencion y representacion completa de ceros y las
figuras del informe quedan para la etapa de documentacion.

Los dos arreglos internos ocupan 328 o 344 bytes por instancia, mas el indice y
posible alineacion. La cabecera contiene ademas las tablas constantes de los cuatro
perfiles; su ubicacion efectiva depende del compilador. Dos bloques float32 de 1024
muestras agregarian 8192 bytes fuera del objeto. No se aprovecha la simetria para
reducir multiplicaciones: se prioriza una implementacion directa verificable.
