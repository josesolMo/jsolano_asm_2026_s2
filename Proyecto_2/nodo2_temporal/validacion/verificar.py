"""Compila C++ y valida respuesta y continuidad contra scipy.signal.lfilter."""
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
from scipy import signal

from disenar import RAIZ, FS, respuesta


def main():
    # Las tolerancias se fijan antes de las pruebas para entradas de pico <= 1.
    atol, rtol = 2e-6, 2e-5
    compilador = shutil.which('g++') or shutil.which('clang++')
    if not compilador:
        raise RuntimeError('Se requiere g++ o clang++')
    perfiles = json.loads((RAIZ / 'diseno.json').read_text(encoding='utf-8'))
    rng = np.random.default_rng(2026)
    t = np.arange(8197) / FS
    impulso = np.zeros(len(t), dtype=np.float32)
    impulso[1023] = 1
    # El impulso junto al limite de bloque revela reinicios accidentales de estado.
    senales = {
        'impulso_frontera': impulso,
        'escalon': (np.arange(len(t)) >= 1023).astype(np.float32),
        'ruido': rng.uniform(-1, 1, len(t)).astype(np.float32),
        'multitono': ((np.sin(2*np.pi*440*t) + .55*np.sin(2*np.pi*880*t)
                       + .3*np.sin(2*np.pi*1320*t)) / 1.85).astype(np.float32),
        'mezcla_p1': (.6*np.sin(2*np.pi*1000*t) + .3*np.sin(2*np.pi*2500*t)).astype(np.float32),
        'armonico_audio': np.sin(2*np.pi*2640*t).astype(np.float32),
        'transitoria': (np.exp(-5*(t % 1)) * (np.sin(2*np.pi*2*t) > 0)
                        * np.sin(2*np.pi*600*t)).astype(np.float32),
    }
    resultados = {}
    # Todo binario de prueba queda fuera del repositorio y se elimina al terminar.
    with tempfile.TemporaryDirectory() as carpeta:
        tmp = Path(carpeta)
        exe = tmp / 'filtro.exe'
        subprocess.run([compilador, '-std=c++11', '-O2', '-Wall', '-Wextra', '-pedantic',
                        '-ffp-contract=off', str(RAIZ / 'validacion/ejecutar_filtro.cpp'),
                        '-o', str(exe)], check=True)
        for nombre, perfil in perfiles.items():
            b = np.asarray(perfil['coeficientes'], dtype=np.float32)
            rizado, atenuacion = respuesta(b, [1], perfil['fp_hz'], perfil['fst_hz'])
            assert rizado <= .5 and atenuacion >= 40, nombre
            np.testing.assert_array_equal(b, b[::-1])
            maximo, casos = 0.0, 0
            for senal_nombre, x in senales.items():
                x.tofile(tmp / 'entrada.bin')
                referencia = signal.lfilter(b.astype(float), [1.0], x.astype(float))
                monolitica = None
                for bloque in (len(x), 1024, 257, 1):
                    for en_sitio in (False, True):
                        subprocess.run([str(exe), nombre, str(tmp/'entrada.bin'),
                                        str(tmp/'salida.bin'), str(bloque), str(int(en_sitio))], check=True)
                        y = np.fromfile(tmp/'salida.bin', dtype=np.float32)
                        np.testing.assert_allclose(y, referencia, atol=atol, rtol=rtol,
                                                   err_msg=f'{nombre}/{senal_nombre}/{bloque}')
                        # Cambiar la particion y usar el mismo bufer debe ser identico bit a bit.
                        if monolitica is None:
                            monolitica = y
                        np.testing.assert_array_equal(y, monolitica)
                        maximo = max(maximo, float(np.max(np.abs(y - referencia))))
                        casos += 1
            resultados[nombre] = dict(casos=casos, error_absoluto_maximo=maximo,
                                      rizado_db=rizado, atenuacion_db=atenuacion)
    informe = dict(atol=atol, rtol=rtol, numpy=np.__version__, resultados=resultados)
    (RAIZ/'validacion/resultados.json').write_text(json.dumps(informe, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(informe, indent=2))


if __name__ == '__main__':
    main()
