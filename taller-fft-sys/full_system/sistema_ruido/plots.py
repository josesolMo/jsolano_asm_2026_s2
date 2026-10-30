"""Funciones para generar las figuras del informe."""
from __future__ import annotations

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal as sg
from .config import FS, N, BIN_HZ, TRANS_HZ, ATEN_DB
from .scenarios import escenario
from .temporal_filters import orden_iir


def fig_mapa_frecuencias(archivo: Path):
    """Plan de frecuencias: banda util, interferencia y transicion del filtro por prueba."""
    pids = ["P1", "P2", "P3", "P4", "P5"]
    fig, ax = plt.subplots(figsize=(11, 4.8))
    for i, pid in enumerate(pids):
        esc = escenario(pid)
        y = len(pids) - 1 - i
        lo, hi = esc["banda_util"]
        ri = esc["rango_int"]
        ax.barh(y + .27, hi - lo, left=lo, height=.22, color="tab:green", label="Banda util" if i == 0 else None)
        ax.barh(y, max(ri[1] - ri[0], 45), left=ri[0], height=.22, color="tab:red",
                label="Interferencia principal" if i == 0 else None)
        ax.barh(y - .27, TRANS_HZ, left=hi, height=.22, color="gray",
                label="Transicion del filtro (FIR/IIR)" if i == 0 else None)
        if esc["senal"]["tipo"] == "multitono":
            ax.plot(esc["senal"]["f"], [y + .27] * len(esc["senal"]["f"]), "k|", ms=11,
                    label="Tonos utiles" if i == 0 else None)
        ax.text(60, y + .47, f"{pid}: {esc['nombre']}", ha="left", va="center", fontsize=8, weight="bold")
    ax.axvline(FS / 2, color="k", ls="--", lw=1)
    ax.text(FS / 2 - 60, len(pids) - .45, "Nyquist", ha="right", fontsize=8)
    ax.set_yticks([])
    ax.set(xlim=(0, FS / 2 + 200), ylim=(-.5, len(pids) - .3), xlabel="Frecuencia (Hz)",
           title=f"Plan de frecuencias (fs = {FS} Hz, N = {N}, resolucion = {BIN_HZ:.3f} Hz/bin)")
    ax.legend(loc="upper center", fontsize=8, bbox_to_anchor=(0.5, -0.13), ncol=4, frameon=False)
    fig.tight_layout()
    fig.savefig(archivo, dpi=130)
    plt.close(fig)


def fig_filtros(res: dict, archivo: Path):
    """Respuesta en frecuencia, polos/ceros y retardo de grupo del FIR y del IIR de una prueba."""
    b, sos, esc = res["b"], res["sos"], res["esc"]
    fp = esc["banda_util"][1]
    fst = fp + TRANS_HZ
    b_i, a_i = sg.sos2tf(sos)
    w, Hf = sg.freqz(b, [1.0], worN=4096, fs=FS)
    _, Hi = sg.freqz(b_i, a_i, worN=4096, fs=FS)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    ax[0, 0].plot(w, 20 * np.log10(np.maximum(np.abs(Hf), 1e-6)), label=f"FIR (orden {len(b) - 1})")
    ax[0, 0].plot(w, 20 * np.log10(np.maximum(np.abs(Hi), 1e-6)), label=f"IIR eliptico (orden {orden_iir(sos)})")
    ax[0, 0].axvspan(0, fp, color="green", alpha=.1)
    ax[0, 0].axvspan(fst, FS / 2, color="red", alpha=.08)
    ax[0, 0].axhline(-ATEN_DB, color="k", ls=":", lw=1)
    ax[0, 0].set(ylim=(-90, 5), xlabel="Frecuencia (Hz)", ylabel="|H| (dB)", title="Respuesta en magnitud")
    ax[0, 0].legend(fontsize=8)
    th = np.linspace(0, 2 * np.pi, 400)
    for a_, (z, p), titulo in [(ax[0, 1], (np.roots(b), np.zeros(len(b) - 1)),
                                f"FIR: ceros y {len(b) - 1} polos en z = 0"),
                               (ax[1, 0], sg.sos2zpk(sos)[:2], "IIR: polos (x) y ceros (o)")]:
        a_.plot(np.cos(th), np.sin(th), "k", lw=.8)
        a_.plot(z.real, z.imag, "o", mfc="none")
        a_.plot(p.real, p.imag, "x")
        a_.set(aspect="equal", title=titulo, xlabel="Re(z)", ylabel="Im(z)")
        a_.axhline(0, color="gray", lw=.5)
        a_.axvline(0, color="gray", lw=.5)
    f_gd = np.linspace(1, FS / 2 - 1, 2000)
    ax[1, 1].plot(f_gd, sg.group_delay((b, [1.0]), w=f_gd, fs=FS)[1], label="FIR")
    ax[1, 1].plot(f_gd, sg.group_delay((b_i, a_i), w=f_gd, fs=FS)[1], label="IIR")
    ax[1, 1].set(xlabel="Frecuencia (Hz)", ylabel="Retardo de grupo (muestras)", title="Retardo de grupo")
    ax[1, 1].legend(fontsize=8)
    fig.suptitle(f"Filtros candidatos de {esc['id']} - banda de paso hasta {fp} Hz, rechazo desde {fst} Hz "
                 "(ROC causal: |z| > radio maximo de polos)", fontsize=10)
    fig.tight_layout()
    fig.savefig(archivo, dpi=130)
    plt.close(fig)


def fig_espectros(resultados: dict, archivo: Path):
    """Espectro de limpia, contaminada y las tres salidas por prueba."""
    fig, axs = plt.subplots(2, 3, figsize=(16, 8.5))
    for ax, (pid, res) in zip(axs.ravel(), resultados.items()):
        esc = res["esc"]
        datos = [("Limpia", res["limpia"], "k", 1.6), ("Contaminada (ADC)", res["adc"], "0.6", 1.0),
                 ("FIR", res["salidas"]["FIR"], "C0", 1.0), ("IIR", res["salidas"]["IIR"], "C1", 1.0),
                 ("Espectral", res["salidas"]["ESPECTRAL"], "C3", 1.0)]
        for nombre, x, c, lw in datos:
            f, P = sg.welch(x, FS, nperseg=2048)
            ax.plot(f, 10 * np.log10(P + 1e-16), color=c, lw=lw, label=nombre)
        ax.axvspan(*esc["banda_util"], color="green", alpha=.10)
        ax.axvspan(*esc["rango_int"], color="red", alpha=.07)
        pmax = 10 * np.log10(sg.welch(res["limpia"], FS, nperseg=2048)[1].max())
        ax.set(title=f"{pid}: {esc['nombre']}", xlabel="Frecuencia (Hz)", ylabel="PSD (dB)",
               xlim=(0, FS / 2), ylim=(pmax - 100, pmax + 8))
        ax.title.set_fontsize(8.5)
    axs[0, 0].legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(archivo, dpi=130)
    plt.close(fig)
