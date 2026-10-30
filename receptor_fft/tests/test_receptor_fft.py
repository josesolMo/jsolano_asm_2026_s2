"""Pruebas del receptor: FFT contra la definición, criterios y trama.

Ejecutar desde la raíz: python -m unittest discover -s receptor_fft/tests -t . -v
"""

import unittest

import numpy as np

from receptor_fft.src.receptor_fft import (
    FLAG_BANDA_UTIL, FLAG_INTERFERENTE, FS, N_BLOQUE, SALTO, Receptor, candidatos_bloque,
    desempaquetar, empaquetar, espectro_potencia, procesar_senal, validar_contra_emisor,
    ventana_hann)

DF = FS / N_BLOQUE


def tono(f, a=0.3, n=N_BLOQUE, fase=0.4):
    return a * np.sin(2 * np.pi * f * np.arange(n) / FS + fase)


class PruebasFFT(unittest.TestCase):
    def test_fft_igual_a_dft_por_definicion(self):
        # La FFT usada (o la de la biblioteca del ESP32) se valida contra la DFT directa.
        rng = np.random.default_rng(0)
        for n in (64, 256, 1024):
            x = rng.standard_normal(n)
            xv = (x - x.mean()) * ventana_hann(n)
            k = np.arange(n // 2 + 1)[:, None]
            dft = (xv * np.exp(-2j * np.pi * k * np.arange(n) / n)).sum(axis=1)
            np.testing.assert_allclose(espectro_potencia(x), np.abs(dft) ** 2,
                                       rtol=1e-9, atol=1e-9)

    def test_rechaza_bloques_no_potencia_de_dos(self):
        for n in (1000, 1023, 4):
            with self.assertRaises(ValueError):
                espectro_potencia(np.zeros(n))


class PruebasCriterios(unittest.TestCase):
    def test_frecuencia_y_amplitud(self):
        # En bin exacto y entre bins: error < 0.3 Hz (Δf/50) y amplitud en ±1.5 dB.
        rng = np.random.default_rng(1)
        for f in (2500.0, 2507.8125, 2513.7, 60.0):
            x = tono(f) + 0.01 * rng.standard_normal(N_BLOQUE)
            ok, _ = candidatos_bloque(x)
            c = min(ok, key=lambda c: abs(c['f_hz'] - f))
            self.assertLess(abs(c['f_hz'] - f), 0.3)
            self.assertLess(abs(20 * np.log10(c['amplitud'] / 0.3)), 1.5)

    def test_solo_ruido_no_genera_candidatos(self):
        rng = np.random.default_rng(2)
        for _ in range(300):
            self.assertEqual(candidatos_bloque(rng.standard_normal(N_BLOQUE))[0], [])

    def test_c2_rechaza_frecuencias_muy_bajas(self):
        # 20 Hz cae en el bin 1: indistinguible de la fuga del DC con N = 1024.
        ok, rech = candidatos_bloque(tono(20.0) + 1.0)
        self.assertEqual(ok, [])
        self.assertTrue(any(r['falla'].startswith('C2') for r in rech))

    def test_c1_umbral(self):
        # Tono 10 dB por debajo de lo detectable en un bloque: no debe aceptarse.
        rng = np.random.default_rng(3)
        a = np.sqrt(2 * 10 ** (-15 / 10))  # TNR = -15 dB con sigma = 1
        ok, _ = candidatos_bloque(tono(3000.0, a) + rng.standard_normal(N_BLOQUE))
        self.assertEqual(ok, [])


class PruebasPersistencia(unittest.TestCase):
    def _bloques(self, presencia):
        rec = Receptor()
        rng = np.random.default_rng(4)
        salidas = []
        for presente in presencia:
            x = 0.01 * rng.standard_normal(N_BLOQUE) + (tono(3100.0) if presente else 0)
            salidas.append(any(abs(s['f_hz'] - 3100) < DF for s in rec.procesar(x)))
        return salidas

    def test_necesita_tres_de_cuatro(self):
        self.assertEqual(self._bloques([1, 1, 0, 0, 0]), [False] * 5)
        self.assertEqual(self._bloques([1, 1, 1])[-1], True)
        self.assertEqual(self._bloques([1, 0, 1, 1])[-1], True)

    def test_histeresis(self):
        # Una sola ausencia no libera la frecuencia; dos ausencias seguidas tampoco
        # (quedan 2 de 4); con tres ausencias sí se libera.
        s = self._bloques([1, 1, 1, 0, 0, 0])
        self.assertEqual(s, [False, False, True, True, True, False])

    def test_clasificacion_banda_util(self):
        t = np.arange(N_BLOQUE + 3 * SALTO) / FS
        x = 0.3 * np.sin(2 * np.pi * 880 * t) + 0.2 * np.sin(2 * np.pi * 2500 * t)
        ultimo = procesar_senal(x)[-1]
        flags = {round(s['f_hz']): s['flags'] for s in ultimo}
        self.assertEqual(flags[880], FLAG_BANDA_UTIL)
        self.assertEqual(flags[2500], FLAG_INTERFERENTE)
        self.assertTrue(ultimo[0]['flags'] & FLAG_INTERFERENTE)  # interferentes primero


class PruebasEmisor(unittest.TestCase):
    def test_tabla_del_emisor(self):
        # Recibe 440 y 2500 (con error pequeño), falta 880 y llega un 3700 ajeno.
        validadas = [{'f_hz': 440.2, 'amplitud': 0.3}, {'f_hz': 2507.0, 'amplitud': 0.2},
                     {'f_hz': 3700.0, 'amplitud': 0.1}]
        reporte, extra = validar_contra_emisor(validadas, (440.0, 880.0, 2500.0), 15.625)
        self.assertEqual([r['recibida'] for r in reporte], [True, False, True])
        self.assertAlmostEqual(reporte[2]['error_hz'], 7.0)
        self.assertEqual([e['f_hz'] for e in extra], [3700.0])

    def test_fuera_de_tolerancia(self):
        # A 25 Hz de lo esperado (1 % de 2500 Hz) ya no se acepta como del emisor.
        reporte, extra = validar_contra_emisor([{'f_hz': 2525.0, 'amplitud': 0.2}],
                                               (2500.0,), 15.625)
        self.assertFalse(reporte[0]['recibida'])
        self.assertEqual(len(extra), 1)


class PruebasTrama(unittest.TestCase):
    def test_ida_y_vuelta(self):
        datos = [{'f_hz': 2513.75, 'snr_local_db': 41.4, 'flags': FLAG_INTERFERENTE},
                 {'f_hz': 60.0, 'snr_local_db': 25.0, 'flags': 0x03}]
        trama = empaquetar(7, datos)
        self.assertEqual(len(trama), 5 + 4 * 2 + 2)
        sec, leidos = desempaquetar(trama)
        self.assertEqual(sec, 7)
        self.assertEqual([d['f_hz'] for d in leidos], [2513.75, 60.0])
        self.assertEqual([d['snr_local_db'] for d in leidos], [41, 25])

    def test_crc_detecta_errores(self):
        trama = bytearray(empaquetar(1, [{'f_hz': 2500.0, 'snr_local_db': 30, 'flags': 1}]))
        for i in range(2, len(trama) - 2):
            corrupta = bytearray(trama)
            corrupta[i] ^= 0x10
            with self.assertRaises(ValueError):
                desempaquetar(bytes(corrupta))


if __name__ == '__main__':
    unittest.main()
