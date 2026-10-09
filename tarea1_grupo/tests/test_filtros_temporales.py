"""Pruebas de causalidad, estado y equivalencia numérica del FIR."""

import unittest
from pathlib import Path

import numpy as np
from scipy import signal

from tarea1_grupo.src.analisis_temporal import leer_pcm16, validar
from tarea1_grupo.src.filtros_temporales import ATOL, RTOL, disenar_filtro, filtrar_manual, medir_respuesta


class PruebasTemporal(unittest.TestCase):
    def test_especificaciones_y_float32(self):
        # La cuantización debe preservar las especificaciones del filtro acordado.
        b = disenar_filtro()
        self.assertEqual(len(b), 41)
        np.testing.assert_allclose(b, b[::-1], atol=1e-15, rtol=0)
        for coeficientes in (b, b.astype(np.float32)):
            mediciones = medir_respuesta(coeficientes)
            self.assertLessEqual(mediciones['rizado_dB'], .5)
            self.assertGreaterEqual(mediciones['atenuacion_rechazo_dB'], 40)

    def test_impulso(self):
        # La cola nula detecta errores en el manejo del historial inicial.
        b = disenar_filtro()
        x = np.zeros(97)
        x[0] = 1
        y, _ = validar(x, b, 'impulso', bloque=17)
        np.testing.assert_allclose(y, np.r_[b, np.zeros(56)], atol=ATOL, rtol=RTOL)

    def test_tres_casos(self):
        # 96000 no es múltiplo de 1024: el último bloque tiene 768 muestras.
        raiz = Path(__file__).resolve().parents[1]
        for caso in range(1, 4):
            with self.subTest(caso=caso):
                entrada = leer_pcm16(raiz / f'pruebas/caso_{caso}/03_entrada_contaminada.wav')
                _, registro = validar(entrada, disenar_filtro(), str(caso))
                self.assertEqual(registro['ultimo_bloque_muestras'], 768)

    def test_bloques_menores_que_estado_y_vacios(self):
        # Bloques de una muestra y vacíos no deben perder las entradas anteriores.
        b = disenar_filtro()
        x = np.random.default_rng(12).normal(size=103)
        estado = None
        partes = []
        for muestra in x:
            parte, estado = filtrar_manual([muestra], b, estado)
            partes.append(parte)
            vacio, siguiente = filtrar_manual([], b, estado)
            self.assertEqual(len(vacio), 0)
            np.testing.assert_array_equal(estado, siguiente)
        np.testing.assert_allclose(np.concatenate(partes), signal.lfilter(b, [1], x), atol=ATOL, rtol=RTOL)


if __name__ == '__main__':
    # unittest mantiene las pruebas ejecutables sin dependencias adicionales.
    unittest.main()
