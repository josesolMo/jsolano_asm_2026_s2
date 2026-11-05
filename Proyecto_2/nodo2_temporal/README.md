# Filtro temporal del nodo 2

Primera implementacion independiente de SPI y DAC. Reutiliza las especificaciones
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
mayor que 1: `diseno.json` incluye la cota sum(abs(b)). El escalado, el retiro de DC
del ADC, el margen de amplitud y la saturacion para DAC se resolveran en la integracion.
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
