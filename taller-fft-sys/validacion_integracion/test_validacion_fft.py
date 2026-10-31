"""Pruebas reproducibles de deteccion FFT (sin hardware)."""
import unittest
import numpy as np

from validacion_fft import (FS, N, BIN_HZ, detectar_picos_fft,
                            validar_componentes, validar_modos)


class PruebasFFT(unittest.TestCase):
    def test_frecuencia_y_nyquist(self):
        self.assertEqual(FS, 16000)
        self.assertEqual(N, 1024)
        self.assertEqual(BIN_HZ, 15.625)
        self.assertEqual(len(np.fft.rfftfreq(N, 1 / FS)), N // 2 + 1)

    def test_seno_1000(self):
        t = np.arange(N) / FS
        x = np.sin(2 * np.pi * 1000 * t)
        filas = validar_componentes(x, [1000], "seno", 0)
        self.assertTrue(filas[0]["cumple"])

    def test_seno_2500_y_offset_dc(self):
        t = np.arange(N) / FS
        x = 128 + 80 * np.sin(2 * np.pi * 2500 * t)
        filas = validar_componentes(x, [2500], "dac simulado", 0)
        self.assertTrue(filas[0]["cumple"])

    def test_multitono(self):
        t = np.arange(N) / FS
        x = (np.sin(2 * np.pi * 440 * t)
             + .55 * np.sin(2 * np.pi * 880 * t + np.deg2rad(35))
             + .30 * np.sin(2 * np.pi * 1320 * t - np.deg2rad(50)))
        filas = validar_componentes(x, [440, 880, 1320], "multitono", 0)
        self.assertTrue(all(f["cumple"] for f in filas))

    def test_detecta_frecuencia_incorrecta(self):
        t = np.arange(N) / FS
        x = np.sin(2 * np.pi * 1000 * t)
        filas = validar_componentes(x, [2500], "prueba negativa", 0)
        self.assertFalse(filas[0]["cumple"])

    def test_modes_sistema_existente(self):
        filas = validar_modos()
        self.assertEqual(len(filas), 10)
        self.assertTrue(all(f["cumple"] for f in filas), filas)

    def test_argumento_longitud_invalida(self):
        with self.assertRaises(ValueError):
            detectar_picos_fft(np.zeros(N - 1))


if __name__ == "__main__":
    unittest.main()
