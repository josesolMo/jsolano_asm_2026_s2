// Genera un flujo con el codigo REAL del Nodo 1 (Proyecto_2/nodo1_esp32s3/trama.h): muestras, diagnostico y
// una trama de espectro. Escribe tambien la salida que debe producir el parser (formato de host_link).
// Uso: gen_nodo1 flujo.bin esperado.txt [bloques]
#include "../../nodo1_esp32s3/trama.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

struct Frec { float f_hz, amplitud, snr_db; uint8_t flags; };

int main(int argc, char** argv) {
  const int bloques = argc > 3 ? atoi(argv[3]) : 12;
  FILE* fo = fopen(argv[1], "wb"); FILE* fe = fopen(argv[2], "w");
  uint32_t lcg = 1, analisis = 0;
  std::vector<uint8_t> t(trama::tamano(1024));
  for (int s = 0; s < bloques; ++s) {
    int16_t m[1024];
    for (int i = 0; i < 1024; ++i) { lcg = lcg * 1664525u + 1013904223u; m[i] = int16_t(lcg >> 20); }   // 0..4095 como el ADC
    if (s == 3) { m[10] = 0x5AA5; m[11] = 0x5BA5; m[12] = 0x0400; }       // imita cabeceras dentro de los datos
    for (int a = 0; a < 2; ++a, ++analisis) {                          // 2 diagnosticos por bloque (cada 32 ms)
      Frec fr[8]; int n = (s * 2 + a) % 9;                                // 0..8 frecuencias
      for (int i = 0; i < n; ++i) fr[i] = {440.25f * (i + 1), 0.0123f * (i + 1), 12.3f + i, uint8_t(1 + (i % 3))};
      uint8_t d[trama::tamanoDiagnostico(8)];
      const size_t l = trama::empaquetarDiagnostico(analisis, uint16_t(5000 + s), fr, n, d);
      fwrite(d, 1, l, fo);
      fprintf(fe, "DIAG %u %u %d", analisis, 5000 + s, n);
      for (int i = 0; i < n; ++i) fprintf(fe, " %.2f,%.4f,%.1f,%u", fr[i].f_hz, fr[i].amplitud, fr[i].snr_db, fr[i].flags);
      fprintf(fe, "\n");
    }
    if (s == 5) {                                                         // espectro a peticion: debe ignorarse
      static float x[1024], p[513]; static uint8_t e[trama::tamanoEspectro(1024)];
      for (int i = 0; i < 1024; ++i) x[i] = float(i) * 0.5f;
      const size_t l = trama::empaquetarEspectro(analisis, x, p, 1024, e); fwrite(e, 1, l, fo);
    }
    const size_t l = trama::empaquetar(uint16_t(s), m, 1024, t.data());
    fwrite(t.data(), 1, l, fo);
    uint32_t h = 2166136261u; for (int i = 0; i < 1024; ++i) { h ^= uint16_t(m[i]); h *= 16777619u; }
    fprintf(fe, "PCM %d %08x\n", s, h);
  }
  fclose(fo); fclose(fe);
  return 0;
}
