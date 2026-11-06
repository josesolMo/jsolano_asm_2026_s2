"""Prueba de 'recomendar' con la cadena completa en el PC: escenario -> codigo del Nodo 1
(enlace_host.cpp, con la banda util de cada escenario) -> parser del Nodo 3 -> recomendar().

Lo esperado es el metodo con mayor dSNR en la simulacion del sistema (full_system,
taller-fft-sys/resultados_sistema/metricas.csv, media de las repeticiones):
    P1 FIR 28.4 / espectral 38.7 dB   P2 FIR 34.1 / espectral 12.5 dB   P3 FIR 4.4 / espectral 10.9 dB
    P4 FIR 0.2 / espectral 1.6 dB     P5 FIR 2.6 / espectral 10.1 dB
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

FW = Path(__file__).resolve().parents[1]
RAIZ = FW.parent
sys.path.insert(0, str(RAIZ / "nodo1_esp32s3" / "validacion"))
from probar_enlace import senal_adc   # noqa: E402  (genera el escenario como lo ve el ADC)

# escenario: (banda util del Nodo 1, perfil FIR que elige 'scen', modo esperado)
CASOS = {"P1": ((900, 1100), 0, "filter"), "P2": ((340, 1420), 1, "fir"), "P3": ((150, 2700), 2, "filter"),
         "P4": ((300, 900), 3, "filter"), "P5": ((340, 1420), 1, "filter")}


def main():
    tmp = Path(tempfile.mkdtemp())
    n1, rec = tmp / "enlace_host", tmp / "host_recomendar"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-o", str(n1),
                    str(RAIZ / "nodo1_esp32s3/validacion/enlace_host.cpp")], check=True)
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(rec),
                    str(FW / "tests/host_recomendar.cpp"), str(FW / "communication.cpp"),
                    str(FW / "spectral_filter.cpp"), str(FW / "fft_processing.cpp")], check=True)
    fallas = 0
    for esc, ((lo, hi), perfil, esperado) in CASOS.items():
        adc, _ = senal_adc(esc)
        adc.tofile(tmp / "codigos.bin")
        subprocess.run([str(n1), str(tmp / "codigos.bin"), str(tmp / "flujo.bin"), str(lo), str(hi)],
                       check=True, capture_output=True)
        out = subprocess.run([str(rec), str(tmp / "flujo.bin"), str(lo), str(hi), str(perfil)],
                             check=True, capture_output=True, text=True).stdout
        conteo = dict(kv.split("=") for kv in out.split("CONTEO ")[1].split())
        ganador = "fir" if int(conteo["fir"]) > int(conteo["filter"]) else "filter"
        ok = ganador == esperado
        fallas += not ok
        print(f"{esc}: recomienda {ganador} en {conteo[ganador]}/{int(conteo['fir']) + int(conteo['filter'])} "
              f"bloques (esperado {esperado}) {'OK' if ok else 'FALLA'}")
        print("    " + "\n    ".join(out.strip().split("\n")[:-1]))
    print("RESULTADO RECOMENDAR:", "OK" if not fallas else f"{fallas} con fallas")
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
