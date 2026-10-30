"""Compila la logica real del firmware y contrasta sus muestras con el prototipo."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np

# Las referencias se importan del proyecto, sin regenerar sus figuras ni resultados.
CARPETA = Path(__file__).resolve().parent
sys.path.insert(0, str(CARPETA.parents[1] / "full_system"))
from sistema_ruido.config import FS, L
from sistema_ruido.scenarios import escenario
from sistema_ruido.signals import fade, generar_senal


def comprobar(condicion, mensaje):
    # Fallar explicitamente mantiene las comprobaciones incluso al ejecutar Python con -O.
    if not condicion:
        raise AssertionError(mensaje)


def amplitudes(x, frecuencias, inicio):
    # Ajustar senos y cosenos evita fuga cuando las frecuencias no caen en bins enteros.
    t = (np.arange(len(x)) + inicio) / FS
    matriz = np.column_stack([funcion(2 * np.pi * f * t)
                              for f in frecuencias for funcion in (np.sin, np.cos)])
    coeficientes = np.linalg.lstsq(matriz, x - x.mean(), rcond=None)[0]
    return np.hypot(coeficientes[::2], coeficientes[1::2])


def pico_fft(x):
    # Quitar DC y aplicar Hann permite localizar la componente dominante sin el sesgo 128.
    x = x - x.mean()
    potencia = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    return np.fft.rfftfreq(len(x), 1 / FS)[np.argmax(potencia)]


def main():
    # El ejecutable y las muestras viven fuera del repositorio y se eliminan al terminar.
    compilador = shutil.which(os.environ.get("CXX", "g++"))
    if not compilador:
        raise RuntimeError("Falta g++; instale un compilador C++ o configure CXX.")
    with tempfile.TemporaryDirectory(prefix="emisor_fft_") as temporal:
        ejecutable = Path(temporal) / ("exportar.exe" if os.name == "nt" else "exportar")
        subprocess.run([compilador, "-std=c++17", "-O2", "-Wall", "-Wextra", "-pedantic",
                        str(CARPETA / "exportar.cpp"), "-o", str(ejecutable)], check=True)
        subprocess.run([str(ejecutable), temporal], check=True)
        referencias = [dict(tipo="multitono", f=[1000], a=[1], fase=[0]),
                       dict(tipo="multitono", f=[2500], a=[1], fase=[0]),
                       escenario("P2")["senal"], escenario("P4")["senal"],
                       escenario("P3")["senal"]]
        for modo, parametros in enumerate(referencias, 1):
            # Comparar cada muestra contra Python verifica fases, duracion y envolventes.
            codigos = np.fromfile(Path(temporal) / f"modo_{modo}.bin", dtype=np.uint8)
            comprobar(len(codigos) == L + 4096, "Longitud de secuencia incorrecta")
            comprobar(np.all(codigos[L:] == 128), "La cola no queda en el centro")
            comprobar(48 <= codigos.min() <= codigos.max() <= 208, "Rango incorrecto")
            comprobar(codigos[0] == codigos[L - 1] == 128, "Rampa global incorrecta")
            t = np.arange(L) / FS
            referencia = fade(generar_senal(t=t, **parametros))
            ideal = 128 + 80 * referencia / np.max(np.abs(referencia))
            error = np.abs(codigos[:L].astype(float) - ideal)
            comprobar(error.max() <= .50001, f"Modo {modo}: error mayor que medio LSB")
            x = codigos[:L].astype(float) - 128
            if modo <= 3:
                # El intervalo central excluye las rampas; tolerancia de pico: un bin FFT.
                inicio, fin = FS // 4, L - FS // 4
                tramo = x[inicio:fin]
                frecuencias = parametros["f"]
                comprobar(abs(pico_fft(tramo) - frecuencias[0]) <= FS / len(tramo),
                          "Frecuencia principal incorrecta")
                if modo == 3:
                    medidas = amplitudes(tramo, frecuencias, inicio)
                    razones = medidas / medidas[0]
                    comprobar(np.max(np.abs(razones - [1, .55, .30])) < .01,
                              "Relacion de amplitudes fuera de tolerancia absoluta 0.01")
                    print("Multitono: relaciones medidas", razones)
            elif modo == 4:
                comprobar(abs(pico_fft(x) - 600) <= 2 * FS / L, "Portadora transitoria incorrecta")
                # Los cuartos cerrados de cada periodo de puerta deben quedar en 128.
                cerrada = np.sin(2 * np.pi * 2 * t) < -1e-6
                comprobar(np.all(codigos[:L][cerrada] == 128), "Puerta transitoria incorrecta")
            else:
                # Se comprueba la fundamental de las cuatro notas en un tramo sin ataque.
                for j, frecuencia in enumerate(parametros["notas"]):
                    tramo = x[j * (L // 4) + 320:j * (L // 4) + 16000]
                    comprobar(abs(pico_fft(tramo) - frecuencia) <= FS / len(tramo),
                              f"Frecuencia incorrecta en nota {j + 1}")
            print(f"Modo {modo}: {L} muestras, {L / FS:.3f} s nominales; "
                  f"rango {codigos.min()}..{codigos.max()}; error maximo {error.max():.6f} LSB")
    print("Validacion digital OK. No acredita reloj, DAC analogico ni ausencia de huecos fisicos.")


if __name__ == "__main__":
    # Esta entrada solo ejecuta la comprobacion solicitada y no modifica las referencias.
    main()
