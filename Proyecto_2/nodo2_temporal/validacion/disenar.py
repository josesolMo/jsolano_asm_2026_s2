"""Diseno reproducible de los candidatos del prototipo, sin importar config.py."""
import json
from pathlib import Path

import numpy as np
from scipy import signal

# Las bandas reproducen P1-P5; P2 y P5 comparten el perfil multitono.
RAIZ = Path(__file__).resolve().parents[1]
FS = 16000
PERFILES = {"tono": 1100, "multitono": 1420, "audio": 2700, "transitoria": 900}
RIZADO = 0.5
ATENUACION = 40.0


def respuesta(b, a, fp, fst):
    """La malla incluye ambos bordes y Nyquist para verificar la especificacion."""
    frecuencias = np.unique(np.r_[np.linspace(0, FS / 2, 65537), fp, fst])
    _, h = signal.freqz(b, a, worN=frecuencias, fs=FS)
    magnitud = np.abs(h)
    paso = magnitud[frecuencias <= fp]
    return (float(20 * np.log10(paso.max() / paso.min())),
            float(-20 * np.log10(magnitud[frecuencias >= fst].max())))


def disenar(fp):
    """Busca el menor FIR impar que cumple tambien al cuantizar a float32."""
    fst = fp + 700
    dp = (10 ** (RIZADO / 20) - 1) / (10 ** (RIZADO / 20) + 1)
    ds = 10 ** (-ATENUACION / 20)
    for cantidad in range(11, 301, 2):
        b = signal.remez(cantidad, [0, fp, fst, FS / 2], [1, 0],
                         weight=[1 / dp, 1 / ds], fs=FS).astype(np.float32)
        rizado, atenuacion = respuesta(b, [1], fp, fst)
        if rizado <= RIZADO and atenuacion >= ATENUACION:
            break
    else:
        raise RuntimeError("No se encontro un FIR valido")
    # El IIR es un candidato de comparacion, no una segunda implementacion C++.
    orden, wn = signal.ellipord(fp, fst, RIZADO, ATENUACION, fs=FS)
    sos = signal.ellip(orden, RIZADO, ATENUACION, wn, fs=FS, output="sos")
    polos = signal.sos2zpk(sos)[1]
    # Medir ambos candidatos permite comparar costo sin confundir orden con calidad.
    bi, ai = signal.sos2tf(sos)
    rizado_iir, atenuacion_iir = respuesta(bi, ai, fp, fst)
    tonos = np.array([60, 440, 600, 880, 1000, 1320, 2500, 2640])
    _, h_tonos = signal.freqz(b, [1], worN=tonos, fs=FS)
    return b, dict(fp_hz=fp, fst_hz=fst, orden_fir=len(b) - 1,
                   rizado_db=rizado, atenuacion_db=atenuacion,
                   retardo_ms=1000 * (len(b) - 1) / (2 * FS),
                   mac_por_muestra=len(b),
                   memoria_arreglos_bytes=8 * len(b),
                   cota_ganancia_pico=float(np.abs(b).sum()),
                   orden_iir=int(orden), radio_polos_iir=float(np.abs(polos).max()),
                   rizado_iir_db=rizado_iir, atenuacion_iir_db=atenuacion_iir,
                   ganancia_tonos_db={str(f): float(20*np.log10(abs(h)))
                                      for f, h in zip(tonos, h_tonos)},
                   coeficientes=b.astype(float).tolist())


def main():
    # La cabecera usa suficientes digitos para recuperar exactamente cada float32.
    cabecera = ['#pragma once', '#include <array>', '', 'namespace temporal {',
                '// Generado por validacion/disenar.py; fs=16000 Hz, float32.']
    informe = {}
    for nombre, fp in PERFILES.items():
        b, informe[nombre] = disenar(fp)
        cabecera.append(f'// Perfil {nombre}: paso hasta {fp} Hz, rechazo desde {fp + 700} Hz.')
        cabecera.append(f'constexpr std::array<float, {len(b)}> {nombre.upper()} = {{{{')
        cabecera.extend(f'    {float(valor):.9e}f,' for valor in b)
        cabecera.append('}};')
    cabecera.append('}  // namespace temporal')
    (RAIZ / 'coeficientes.h').write_text('\n'.join(cabecera) + '\n', encoding='utf-8')
    (RAIZ / 'diseno.json').write_text(json.dumps(informe, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: {a: v for a, v in d.items() if a != 'coeficientes'}
                      for k, d in informe.items()}, indent=2))


if __name__ == '__main__':
    main()
