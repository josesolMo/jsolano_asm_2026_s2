"""FIR causal de fase lineal para la sección 2.3."""

import numpy as np
from scipy import signal

FS = 16000
ORDEN = 40
RETARDO = 20
ATOL = RTOL = 1e-10


def disenar_filtro():
    """Diseña los 41 coeficientes sin normalización posterior."""
    # Los pesos equilibran los errores permitidos en paso y rechazo.
    delta_p = (10**(0.5 / 20) - 1) / (10**(0.5 / 20) + 1)
    delta_s = 10**(-40 / 20)
    return signal.remez(41, [0, 1500, 2200, 8000], [1, 0],
                        weight=[1 / delta_p, 1 / delta_s], fs=FS)


def filtrar_manual(entrada, coeficientes, estado=None):
    """Devuelve salida y últimas M entradas; estado=None implica reposo inicial.

    Cada iteración k suma explícitamente b[k]*x[n-k] para todos los n.
    La vectorización sobre n evita un bucle Python por muestra y no delega
    la suma a una rutina de filtrado, convolución o producto matricial.
    """
    # Validar dimensiones evita estados ambiguos al encadenar bloques.
    entrada = np.asarray(entrada, dtype=np.float64)
    coeficientes = np.asarray(coeficientes, dtype=np.float64)
    if entrada.ndim != 1 or coeficientes.ndim != 1 or not coeficientes.size:
        raise ValueError('Entrada y coeficientes deben ser vectores unidimensionales.')
    memoria = len(coeficientes) - 1
    estado = np.zeros(memoria) if estado is None else np.asarray(estado, dtype=np.float64)
    if estado.shape != (memoria,):
        raise ValueError(f'El estado debe contener {memoria} muestras, de antigua a reciente.')
    # El historial representa las entradas anteriores, incluidas las nulas iniciales.
    historial = np.concatenate((estado, entrada))
    salida = np.zeros(len(entrada), dtype=np.float64)
    for k, coeficiente in enumerate(coeficientes):
        salida += coeficiente * historial[memoria - k:memoria - k + len(entrada)]
    nuevo_estado = historial[-memoria:].copy() if memoria else np.empty(0)
    return salida, nuevo_estado


def medir_respuesta(coeficientes):
    """Mide bandas incluyendo sus extremos sobre una rejilla de alta densidad."""
    # Incluir extremos exactos evita omitir los bordes de las especificaciones.
    frecuencias = np.unique(np.r_[np.linspace(0, FS / 2, 262145), 1500, 2200, 2500])
    _, respuesta = signal.freqz(coeficientes, [1.0], worN=frecuencias, fs=FS)
    magnitud = np.abs(respuesta)
    paso = magnitud[frecuencias <= 1500]
    rechazo = magnitud[frecuencias >= 2200]
    return dict(rizado_dB=float(20 * np.log10(paso.max() / paso.min())),
                atenuacion_rechazo_dB=float(-20 * np.log10(rechazo.max())),
                ganancia_2500_dB=float(20 * np.log10(magnitud[frecuencias == 2500][0])),
                ganancia_paso_min=float(paso.min()), ganancia_paso_max=float(paso.max()))
