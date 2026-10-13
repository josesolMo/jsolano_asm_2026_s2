"""Pruebas de reconstrucción, simetría y validación del método espectral."""

import unittest
from pathlib import Path

import numpy as np

from tarea1_grupo.src.filtrado_espectral import (
    G_MIN, estimar_potencia_ruido, ganancia_sustraccion, leer_wav, procesar_bloques,
    reducir_ruido, snr_db, ventana_raiz_hann, verificar_simetria)

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


class PruebasGananciaAdaptativa(unittest.TestCase):
    def test_estimacion_sigue_al_ruido(self):
        # Adaptativo: si la potencia del ruido cambia, la estimación debe cambiar igual.
        rng = np.random.default_rng(2)
        ventana = ventana_raiz_hann(1024)
        for sigma in (0.01, 0.1, 1.0):
            estimaciones, reales = [], []
            for _ in range(50):
                espectro = np.fft.rfft(sigma * rng.standard_normal(1024) * ventana)
                estimaciones.append(np.mean(estimar_potencia_ruido(espectro)[1:-1]))
                reales.append(np.mean(np.abs(espectro[1:-1]) ** 2))
            self.assertAlmostEqual(np.mean(estimaciones) / np.mean(reales), 1, delta=0.1)

    def test_rango_de_ganancia(self):
        # La ganancia debe ser real, de longitud n/2+1 y estar entre G_MIN y 1.
        rng = np.random.default_rng(3)
        espectro = np.fft.rfft(rng.standard_normal(1024))
        g = ganancia_sustraccion(espectro)
        self.assertEqual(g.shape, espectro.shape)
        self.assertFalse(np.iscomplexobj(g))
        self.assertTrue(np.all((g >= G_MIN - 1e-15) & (g <= 1)))

    def test_tono_se_conserva(self):
        # Un tono muy por encima del ruido debe pasar con ganancia cercana a 1.
        rng = np.random.default_rng(4)
        n = np.arange(1024)
        bloque = (np.sin(2 * np.pi * 64 * n / 1024) + 0.01 * rng.standard_normal(1024))
        g = ganancia_sustraccion(np.fft.rfft(bloque * ventana_raiz_hann(1024)))
        self.assertGreater(g[64], 0.99)

    def test_potencia_externa(self):
        # Si otro nodo entrega la potencia de ruido, se usa esa y no la estimada.
        espectro = np.ones(513, dtype=complex)
        g = ganancia_sustraccion(espectro, alfa=1, potencia_ruido=np.full(513, 0.75))
        np.testing.assert_allclose(g, 0.5)

    def test_mejora_en_casos(self):
        # Con ruido de banda ancha (2) y de banda limitada (3) la SNR debe subir claramente.
        for caso, mejora_minima in ((2, 12), (3, 10)):
            carpeta = RAIZ / f'pruebas/caso_{caso}'
            limpia = leer_wav(carpeta / '01_referencia_limpia.wav')
            entrada = leer_wav(carpeta / '03_entrada_contaminada.wav')
            mejora = snr_db(limpia, reducir_ruido(entrada)) - snr_db(limpia, entrada)
            self.assertGreater(mejora, mejora_minima)


if __name__ == '__main__':
    unittest.main()