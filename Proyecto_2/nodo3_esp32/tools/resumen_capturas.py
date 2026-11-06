#!/usr/bin/env python3
"""Promedio y variabilidad de las repeticiones de P1-P5 a partir de las capturas del Nodo 3.

El enunciado (seccion 4) pide al menos tres repeticiones por condicion y reportar promedios y
variabilidad basica. Cada repeticion = una grabacion nueva (g15) reproducida con x y capturada con
graficar_captura.py. Las capturas se ordenan en una carpeta por prueba:

    nodo3_esp32/capturas/P1/captura_....bin
    nodo3_esp32/capturas/P2/...

Uso, desde Proyecto_2/:
    python nodo3_esp32/tools/resumen_capturas.py                     # carpeta por defecto
    python nodo3_esp32/tools/resumen_capturas.py --carpeta otra/ruta --csv resumen.csv

Imprime, por prueba y metodo: n, media y desviacion estandar de la atenuacion en el rango de
interferencia y de la energia util conservada. Las capturas con configuracion distinta dentro de
una misma prueba (banda o perfil FIR) se avisan, porque no son repeticiones de la misma condicion.
La PC solo resume lo que calculo la placa (validacion independiente).
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import graficar_captura as gc  # noqa: E402

MEDIDAS = ("atenuacion en rint (dB)", "energia en banda util conservada (%)")


def leer(ruta: Path):
    trama, _ = gc.buscar_trama(ruta.read_bytes())
    if trama is None:
        return None
    c = gc.decodificar(trama)
    filas = {nombre: (e, f) for nombre, e, f in gc.resumen(c)}
    return c, filas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--carpeta", default=str(gc.CARPETA), help="carpeta con una subcarpeta por prueba (P1..P5)")
    ap.add_argument("--csv", help="guarda la tabla tambien en este CSV")
    a = ap.parse_args()
    raiz = Path(a.carpeta)
    tabla = []
    for prueba in sorted(p for p in raiz.iterdir() if p.is_dir()):
        capturas = sorted(prueba.glob("*.bin"))
        datos = [d for d in (leer(b) for b in capturas) if d is not None]
        if not datos:
            continue
        configs = {(c["banda"], c["rint"], c["perfil"]) for c, _ in datos}
        if len(configs) > 1:
            print(f"AVISO {prueba.name}: {len(configs)} configuraciones distintas (banda/rint/perfil); "
                  "no todas son repeticiones de la misma condicion")
        c0 = datos[0][0]
        for medida in MEDIDAS:
            for j, metodo in enumerate(("espectral", f"FIR {gc.PERFIL.get(c0['perfil'], '?')}")):
                v = np.array([filas[medida][j] for _, filas in datos])
                tabla.append(dict(prueba=prueba.name, medida=medida, metodo=metodo, n=len(v),
                                  media=float(v.mean()), desv=float(v.std(ddof=1)) if len(v) > 1 else float("nan"),
                                  minimo=float(v.min()), maximo=float(v.max())))
    if not tabla:
        sys.exit(f"no hay capturas .bin en subcarpetas de {raiz}")
    print(f"{'prueba':7s}{'medida':40s}{'metodo':16s}{'n':>3s}{'media':>9s}{'desv':>8s}{'min':>8s}{'max':>8s}")
    for t in tabla:
        print(f"{t['prueba']:7s}{t['medida']:40s}{t['metodo']:16s}{t['n']:3d}{t['media']:9.1f}{t['desv']:8.2f}"
              f"{t['minimo']:8.1f}{t['maximo']:8.1f}")
    faltan = sorted({t["prueba"] for t in tabla if t["n"] < 3})
    if faltan:
        print(f"Con menos de 3 repeticiones: {', '.join(faltan)} (el enunciado pide al menos 3)")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(tabla[0]))
            w.writeheader()
            w.writerows(tabla)
        print(f"Tabla guardada en {a.csv}")


if __name__ == "__main__":
    main()
