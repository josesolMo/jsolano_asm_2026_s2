"""Punto de entrada y capa de compatibilidad del archivo original."""

# Importar módulos del sistema de ruido
from sistema_ruido.config import *
from sistema_ruido.signals import *
from sistema_ruido.scenarios import *
from sistema_ruido.acquisition import Diagnostico, NodoAdquisicion
from sistema_ruido.temporal_filters import (
    _medir,
    disenar_fir,
    disenar_iir,
    orden_iir,
    FiltroFIR,
    FiltroIIR,
)
from sistema_ruido.spectral import NodoEspectral
from sistema_ruido.link import (
    SYNC,
    ENCABEZADO,
    CRC_BYTES,
    _tabla_crc16,
    _TABLA,
    crc16,
    empaquetar,
    desempaquetar_flujo,
    disenar_enlace,
    probar_enlace,
)
from sistema_ruido.metrics import potencia_en_rango, alinear, metricas, costo_estimado
from sistema_ruido.pipeline import correr_prueba
from sistema_ruido.verification import dft_directa, verificar_bloques, resumen_filtros
from sistema_ruido.plots import fig_mapa_frecuencias, fig_filtros, fig_espectros
from sistema_ruido.main import REPETICIONES, METODOS, CAMPOS, main


if __name__ == "__main__":
    main()
