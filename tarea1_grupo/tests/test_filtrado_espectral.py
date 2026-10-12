"""Pruebas de reconstrucción, simetría y validación del método espectral."""

import unittest
from pathlib import Path

import numpy as np

from tarea1_grupo.src.filtrado_espectral import (
    leer_wav, procesar_bloques, ventana_raiz_hann, verificar_simetria)

RAIZ = Path(__file__).resolve().parents[1]


class PruebasEspectral(unittest.TestCase):
    def test_ventanas_suman_uno(self):
        # Condición de overlap-add: ventana^2 desplazada n/2 debe sumar 1 en todo punto.
        for n in (64, 256, 1024):
            w2 = ventana_raiz_hann(n) ** 2
            np.testing.assert_allclose(w2[:n // 2] + w2[n // 2:], 1, atol=1e-15)

    def test_reconstruccion_perfecta(self):
        # Con ganancia 1 la cadena completa debe devolver la entrada sin cambios.
        rng = np.random.default_rng(0)
        for n in (64, 256, 1024, 2048):
            for largo in (1000, 4097, 96000):
                x = rng.standard_normal(largo)
                y = procesar_bloques(x, n)
                self.assertEqual(len(y), largo)
                np.testing.assert_allclose(y, x, atol=1e-12, rtol=0)

    def test_reconstruccion_casos(self):
        # Mismo chequeo con las señales contaminadas reales de los tres casos.
        for caso in (1, 2, 3):
            x = leer_wav(RAIZ / f'pruebas/caso_{caso}/03_entrada_contaminada.wav')
            np.testing.assert_allclose(procesar_bloques(x), x, atol=1e-12, rtol=0)

    def test_simetria_conjugada(self):
        # Una ganancia arbitraria pero real y simétrica no debe generar parte imaginaria.
        rng = np.random.default_rng(1)
        bloque = rng.standard_normal(1024)
        g = rng.uniform(0, 1, 513)
        self.assertLess(verificar_simetria(bloque, g), 1e-12)

    def test_validaciones(self):
        # Bloques que no son 2^m y ganancias complejas deben rechazarse.
        x = np.zeros(4000)
        for n in (0, 1, 1000, 1023):
            with self.assertRaises(ValueError):
                procesar_bloques(x, n)
        with self.assertRaises(ValueError):
            procesar_bloques(x, 1024, ganancia=lambda e: np.ones(len(e)) * 1j)


if __name__ == '__main__':
    unittest.main()