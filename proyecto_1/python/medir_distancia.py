import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# CONFIGURACIÓN DE PARÁMETROS
# =============================================================================
PUERTO_SERIAL = 'COM3'
BAUD_RATE = 115200

Fs = 10000.0                       # Frecuencia de muestreo (10 kHz)
VELOCIDAD_SONIDO_CM_S = 3430.0    # Velocidad del sonido en cm/s a 20°C
NUM_MUESTRAS = 600                 # Cantidad de muestras (60 ms totales)
f_tono = 2000.0                    # Frecuencia del emisor piezoeléctrico (2 kHz)
duracion_pulso_s = 0.010           # Duración de la ráfaga (10 ms)

# lag_minimo_eco: Ignoramos las primeras 105 muestras (10.5 ms) en la búsqueda
# del eco para asegurar que la ráfaga directa del buzzer ya se apagó.
lag_minimo_eco = 105               

# =============================================================================
# 1. COMUNICACIÓN SERIAL (CAPTURA DE DATOS)
# =============================================================================
print(f"Conectando a {PUERTO_SERIAL} a {BAUD_RATE} baudios...")
try:
    ser = serial.Serial(PUERTO_SERIAL, BAUD_RATE, timeout=2)
    time.sleep(2)
except Exception as e:
    print(f"Error al abrir el puerto serial: {e}")
    print("Verifica que el Arduino esté conectado y el puerto sea el correcto.")
    exit()

print("Enviando comando de medición ('M')...")
ser.write(b'M')

lecturas = []
print("Recibiendo datos del microcontrolador...")

while len(lecturas) < NUM_MUESTRAS:
    try:
        # Leemos línea por línea, decodificamos y limpiamos espacios/saltos
        linea = ser.readline().decode('utf-8', errors='ignore').strip()
        # Verificamos que sea un número válido antes de agregarlo
        if linea.isdigit() or (linea.startswith('-') and linea[1:].isdigit()):
            lecturas.append(float(linea))
    except ValueError:
        continue

ser.close()
print(f"Se capturaron {len(lecturas)} muestras exitosamente.")

# =============================================================================
# 2. PROCESAMIENTO DIGITAL DE SEÑALES (PDS)
# =============================================================================
# Convertimos la lista de lecturas a un arreglo de NumPy
y = np.array(lecturas)

# --- CORRECCIÓN DE NIVEL DC ---
# Restamos la media de la señal para centrar todo el ruido y las ondas en 0.
y = y - np.mean(y)

# Construcción de la señal patrón (Matched Filter)
# Tono de 2 kHz durante 10 ms con envolvente de ventana Hanning para evitar fugas
N_x = int(Fs * duracion_pulso_s)
t_x = np.arange(N_x) / Fs
x_patron = np.sin(2 * np.pi * f_tono * t_x) * np.hanning(N_x)

# Aplicación de correlación cruzada completa (full)
correlacion = np.correlate(y, x_patron, mode='full')

lags = np.arange(-N_x + 1, len(y))

# Para buscar el eco físico, solo nos interesan los retardos positivos (m >= 0).
# Aislamos la mitad positiva de la correlación.
indice_lag_cero = N_x - 1
correlacion_positiva = correlacion[indice_lag_cero:]

# Buscamos el pico máximo SOLO después de la máscara del disparo inicial
# (de lag_minimo_eco en adelante)
zona_busqueda = correlacion_positiva[lag_minimo_eco:]
if len(zona_busqueda) > 0:
    lag_relativo = np.argmax(np.abs(zona_busqueda))
    lag_detectado = lag_minimo_eco + lag_relativo
else:
    lag_detectado = lag_minimo_eco

# Cálculo final de métricas físicas
tof_s = lag_detectado / Fs
distancia_cm = (tof_s * VELOCIDAD_SONIDO_CM_S) / 2.0

print("\n=========================================")
print(f"RETARDO ESTIMADO (Lag) : {lag_detectado} muestras")
print(f"TIEMPO DE VUELO (ToF)  : {tof_s * 1000:.2f} ms")
print(f"DISTANCIA CALCULADA    : {distancia_cm:.1f} cm")
print("=========================================\n")

# =============================================================================
# 3. GRAFICACIÓN DE RESULTADOS
# =============================================================================
plt.figure(figsize=(10, 8))

# Subtrama 1: Señal Cruda (Centrada)
plt.subplot(2, 1, 1)
plt.plot(y, label="Señal Recibida y[n]", color="tab:blue")
plt.title("Señal Capturada por Micrófono MAX4466")
plt.xlabel("Muestra")
plt.ylabel("Amplitud ADC (Centrada en 0)")
plt.axvline(x=lag_minimo_eco, color='orange', linestyle=':', label="Fin de Ráfaga Directa")
plt.grid(True)
plt.legend(loc="upper right")

# Subtrama 2: Correlación y Detección
plt.subplot(2, 1, 2)
plt.plot(lags, correlacion, color='tab:red', label="Correlación $\phi_{xy}[m]$")
plt.axvline(x=lag_detectado, color='green', linestyle='--', linewidth=2, label=f"Eco Detectado (Lag={lag_detectado})")
plt.title(f"Resultado de Correlación - Distancia: {distancia_cm:.1f} cm")
plt.xlabel("Desfase en Muestras (m)")
plt.ylabel("Amplitud de Correlación")
# Limitamos el eje X a la zona útil para observar mejor los ecos
plt.xlim(-50, NUM_MUESTRAS)
plt.grid(True)
plt.legend(loc="upper right")

plt.tight_layout()
plt.show()