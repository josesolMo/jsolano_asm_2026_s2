import time
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Importación de la FFT desde el script de la Etapa 2
from etapa2_fft_dft import fft_manual

# Ruta de salida
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
OUTPUT_DIR = PROJECT_DIR / "docs" / "imagenes"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def ifft_manual(X):
    """Transformada Inversa de Fourier basada en la FFT manual."""
    X = np.asarray(X, dtype=complex)
    return np.conj(fft_manual(np.conj(X))) / len(X)


def correlacion_directa_manual(x, y):
    """Correlación cruzada en el dominio del tiempo O(N^2)."""
    Nx, Ny = len(x), len(y)
    N_out = Nx + Ny - 1
    R = np.zeros(N_out)
    y_padded = np.pad(y, (Nx - 1, Nx - 1), mode="constant")
    for lag in range(N_out):
        R[lag] = np.sum(x * y_padded[lag : lag + Nx])
    return R


def correlacion_fft_manual(x, y):
    """Correlación cruzada mediante FFT O(N log N)."""
    Nx, Ny = len(x), len(y)
    N_min = Nx + Ny - 1
    # Siguiente potencia de 2 para evitar aliasing circular
    N_fft = 1 << (N_min - 1).bit_length()

    x_pad = np.pad(x, (0, N_fft - Nx))
    y_pad = np.pad(y, (0, N_fft - Ny))

    X = fft_manual(x_pad)
    Y = fft_manual(y_pad)

    # Teorema de Convolución: Y[k] * conj(X[k])
    R_freq = Y * np.conj(X)
    r = ifft_manual(R_freq).real
    return np.concatenate([r[-(Nx - 1):], r[:Ny]])


# ---------------------------------------------------------
# Apartado a): Generación de Señal y Eco
# ---------------------------------------------------------
Fs = 8000  # Frecuencia de muestreo (8 kHz)
t_x = np.linspace(0, 0.02, int(Fs * 0.02), endpoint=False)  # Pulso de 20 ms
x = np.sin(2 * np.pi * 1000 * t_x) * np.hanning(len(t_x))  # Chirp/Pulso de 1kHz

# Crear señal recibida con 2 ecos y ruido blanco (AWGN)
retardo_muestras_1 = 150  # Eco principal
retardo_muestras_2 = 320  # Reflexión secundaria
alfa1, alfa2 = 0.6, 0.25

N_recibida = 600
y = np.zeros(N_recibida)

# Insertar ecos
y[retardo_muestras_1 : retardo_muestras_1 + len(x)] += alfa1 * x
y[retardo_muestras_2 : retardo_muestras_2 + len(x)] += alfa2 * x

# Añadir ruido
np.random.seed(42)
y += np.random.normal(0, 0.15, size=N_recibida)

# Gráfica del apartado a)
fig, axs = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
axs[0].plot(np.arange(len(x)) / Fs * 1000, x, color="tab:blue")
axs[0].set_title("Señal Emitida x[n] (Pulso Acústico 1 kHz)")
axs[0].set_ylabel("Amplitud")
axs[0].grid(True)

axs[1].plot(np.arange(len(y)) / Fs * 1000, y, color="tab:orange")
axs[1].set_title("Señal Recibida y[n] (Ecos Contaminados con Ruido)")
axs[1].set_xlabel("Tiempo (ms)")
axs[1].set_ylabel("Amplitud")
axs[1].grid(True)

plt.tight_layout()
file_eco = OUTPUT_DIR / "etapa3_senial_eco.png"
plt.savefig(file_eco, dpi=300)
plt.close()

# ---------------------------------------------------------
# Apartados b) y c): Correlación Directa vs. FFT
# ---------------------------------------------------------
r_directa = correlacion_directa_manual(x, y)
r_fft = correlacion_fft_manual(x, y)

# Escala de desfases (lags)
lags = np.arange(-(len(x) - 1), len(y))

# Estimación del ToF (Tiempo de Vuelo)
idx_max = np.argmax(r_fft)
retardo_estimado_muestras = lags[idx_max]
tof_estimado_ms = (retardo_estimado_muestras / Fs) * 1000
distancia_m = (343.0 * (retardo_estimado_muestras / Fs)) / 2

print(
    f"Retardo Real: {retardo_muestras_1} muestras | Retardo Estimado: {retardo_estimado_muestras} muestras"
)
print(
    f"ToF Estimado: {tof_estimado_ms:.2f} ms | Distancia Estimada: {distancia_m:.2f} m"
)

# Gráfica de apartados b) y c)
plt.figure(figsize=(9, 4.5))
plt.plot(lags, r_directa, label="Correlación Directa $O(N^2)$", color="tab:gray", alpha=0.7)
plt.plot(lags, r_fft, "--", label="Correlación FFT $O(N \\log_2 N)$", color="tab:red")
plt.axvline(
    x=retardo_estimado_muestras,
    color="green",
    linestyle=":",
    label=f"Detección de Eco (Lag = {retardo_estimado_muestras})",
)
plt.title("Detección de Ecos mediante Correlación Cruzada $\\phi_{xy}[m]$")
plt.xlabel("Desfase en Muestras ($m$)")
plt.ylabel("Amplitud de Correlación")
plt.grid(True, linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()

file_corr = OUTPUT_DIR / "etapa3_correlacion.png"
plt.savefig(file_corr, dpi=300)
plt.close()

# ---------------------------------------------------------
# d) Comparación de Tiempos de Ejecución
# ---------------------------------------------------------
tamanios_N = 2 ** np.arange(5, 11)
tiempos_corr_dir, tiempos_corr_fft = [], []

for N in tamanios_N:
    x_test = np.random.randn(N)
    y_test = np.random.randn(N * 2)

    t0 = time.perf_counter()
    _ = correlacion_directa_manual(x_test, y_test)
    tiempos_corr_dir.append(time.perf_counter() - t0)

    t0 = time.perf_counter()
    _ = correlacion_fft_manual(x_test, y_test)
    tiempos_corr_fft.append(time.perf_counter() - t0)

# Gráfica de apartado d)
plt.figure(figsize=(8, 4.5))
plt.plot(tamanios_N, tiempos_corr_dir, "o-", label="Correlación Directa $O(N^2)$")
plt.plot(tamanios_N, tiempos_corr_fft, "s-", label="Correlación FFT $O(N \\log_2 N)$")
plt.xlabel("Tamaño de Señal Base ($N$)")
plt.ylabel("Tiempo de Ejecución (s)")
plt.title("Rendimiento Computacional: Correlación Directa vs FFT")
plt.xscale("log", base=2)
plt.yscale("log")
plt.grid(True, which="both", linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()

file_tiempos_corr = OUTPUT_DIR / "etapa3_tiempos_correlacion.png"
plt.savefig(file_tiempos_corr, dpi=300)
plt.close()

print("Procesamiento de la Etapa 3 completado con éxito.")
print(f"Imágenes generadas en: {OUTPUT_DIR}")