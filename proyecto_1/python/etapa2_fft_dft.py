import time
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Ruta a la carpeta docs/imagenes/
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
OUTPUT_DIR = PROJECT_DIR / "docs" / "imagenes"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

print(f"Ruta absoluta de guardado: {OUTPUT_DIR}")

# Funciones DFT y FFT
def dft_manual(x):
    N = len(x)
    n = np.arange(N)
    k = n.reshape((N, 1))
    e = np.exp(-2j * np.pi * k * n / N)
    return np.dot(e, x)

def fft_manual(x):
    x = np.asarray(x, dtype=complex)
    N = len(x)
    if N <= 1:
        return x
    if N % 2 != 0:
        raise ValueError("N debe ser potencia de 2")
    pares = fft_manual(x[0::2])
    impares = fft_manual(x[1::2])
    factores = np.exp(-2j * np.pi * np.arange(N // 2) / N)
    return np.concatenate([pares + factores * impares, pares - factores * impares])

# Comparación de Tiempos
tamanios_N = 2**np.arange(4, 11)
tiempos_dft, tiempos_fft = [], []

for N in tamanios_N:
    signal_test = np.random.randn(N)
    t0 = time.perf_counter()
    _ = dft_manual(signal_test)
    tiempos_dft.append(time.perf_counter() - t0)
    
    t0 = time.perf_counter()
    _ = fft_manual(signal_test)
    tiempos_fft.append(time.perf_counter() - t0)

plt.figure(figsize=(8, 5))
plt.plot(tamanios_N, tiempos_dft, "o-", label="DFT Manual $O(N^2)$")
plt.plot(tamanios_N, tiempos_fft, "s-", label="FFT Manual $O(N \\log_2 N)$")
plt.xlabel("Tamaño de Muestra ($N$)")
plt.ylabel("Tiempo de Ejecución (s)")
plt.title("Comparación de Rendimiento: DFT vs FFT")
plt.xscale("log", base=2)
plt.yscale("log")
plt.grid(True, which="both", linestyle="--", alpha=0.6)
plt.legend()
plt.tight_layout()

file_tiempos = OUTPUT_DIR / "etapa2_tiempos_dft_fft.png"
plt.savefig(file_tiempos, dpi=300)
plt.close()

# Verificación de existencia en disco
print(f"Existe {file_tiempos.name}?: {file_tiempos.exists()}")

# Graficar Señal
def analizar_y_graficar(senial, Fs, nombre_senial, nombre_archivo):
    N = len(senial)
    espectro = fft_manual(senial)
    frecuencias = np.fft.fftfreq(N, 1 / Fs)
    mitad = N // 2
    f_pos = frecuencias[:mitad]
    magnitud = np.abs(espectro[:mitad]) / N
    fase = np.angle(espectro[:mitad])
    fase[magnitud < 1e-4] = 0

    fig, axs = plt.subplots(3, 1, figsize=(9, 7))
    t = np.arange(N) / Fs
    axs[0].plot(t * 1000, senial, color="tab:blue")
    axs[0].set_title(f"Señal en el Tiempo: {nombre_senial}")
    axs[0].set_xlabel("Tiempo (ms)")
    axs[0].set_ylabel("Amplitud")
    axs[0].grid(True)

    axs[1].stem(f_pos, magnitud, linefmt="tab:red", markerfmt="ro")
    axs[1].set_title("Espectro de Magnitud")
    axs[1].set_xlabel("Frecuencia (Hz)")
    axs[1].set_ylabel("Magnitud Normalizada")
    axs[1].grid(True)

    axs[2].stem(f_pos, fase, linefmt="tab:green", markerfmt="go")
    axs[2].set_title("Espectro de Fase")
    axs[2].set_xlabel("Frecuencia (Hz)")
    axs[2].set_ylabel("Fase (rad)")
    axs[2].grid(True)

    plt.tight_layout()
    file_img = OUTPUT_DIR / nombre_archivo
    plt.savefig(file_img, dpi=300)
    plt.close()
    print(f"Existe {nombre_archivo}?: {file_img.exists()}")

Fs, N_muestras = 2048, 512
t = np.arange(N_muestras) / Fs
senial_senos = np.sin(2 * np.pi * 100 * t) + 0.5 * np.cos(2 * np.pi * 300 * t)
analizar_y_graficar(senial_senos, Fs, "Suma de Senoides (100Hz + 300Hz)", "etapa2_senos.png")

senial_cuadrada = np.sign(np.sin(2 * np.pi * 50 * t))
analizar_y_graficar(senial_cuadrada, Fs, "Onda Cuadrada (50Hz)", "etapa2_cuadrada.png")

print(
    "Simulaciones de la Etapa 2 completadas. Imágenes guardadas en docs/imagenes/"
)