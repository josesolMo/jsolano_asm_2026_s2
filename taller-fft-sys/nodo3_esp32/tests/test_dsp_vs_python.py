"""Pruebas 3, 4 y 5: el DSP del firmware (C++ en el PC) contra el prototipo Python.

Compila tests/host_dsp.cpp con -DNODO3_HOST (mismo codigo del firmware, FFT radix-2 propia)
y compara. NO ejecuta nada en la ESP32: valida la logica, no el hardware.
Uso (desde taller-fft-sys/):  python nodo3_esp32/tests/test_dsp_vs_python.py
"""
import os, subprocess, sys, tempfile
from pathlib import Path
import numpy as np

RAIZ = Path(__file__).resolve().parents[2]
FW = RAIZ / "nodo3_esp32"
sys.path.insert(0, str(RAIZ / "full_system"))
os.chdir(RAIZ.parent)                         # config.py crea taller-fft-sys/resultados_sistema relativo a la raiz
from sistema_ruido.config import N, HOP, L, N_BLOQUES, FS, ADC_BITS, PICO  # noqa: E402
from sistema_ruido import scenarios, signals                                # noqa: E402
from sistema_ruido.acquisition import NodoAdquisicion                        # noqa: E402
from sistema_ruido.spectral import NodoEspectral                             # noqa: E402
from sistema_ruido.metrics import metricas                                   # noqa: E402

TMP = Path(tempfile.mkdtemp(prefix="nodo3_"))
BIN = TMP / "host_dsp"
fallos = []


def compilar():
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-DNODO3_HOST", "-o", str(BIN),
           str(FW / "tests/host_dsp.cpp"), str(FW / "spectral_filter.cpp"), str(FW / "fft_processing.cpp"), str(FW / "metrics.cpp")]
    subprocess.run(cmd, check=True)


def chequeo(nombre, ok, detalle):
    print(f"[{'OK ' if ok else 'FALLA'}] {nombre}: {detalle}")
    if not ok:
        fallos.append(nombre)


def senal_escenario(pid):
    """Misma generacion que pipeline.correr_prueba, hasta el ADC (lo que enviaria el Nodo 1)."""
    esc = scenarios.escenario(pid)
    rng = np.random.default_rng(esc["semilla"])
    t = np.arange(L) / FS
    limpia = signals.fade(signals.generar_senal(t=t, **esc["senal"]))
    ruido = signals.escalar_para_snr(limpia, signals.generar_ruido(n=L, rng=rng, **esc["ruido"]), esc["snr_db"])
    g = PICO / max(np.abs(limpia).max(), np.abs(limpia + ruido).max())
    limpia, ruidosa = g * limpia, g * (limpia + ruido)
    return esc, limpia, signals.cuantizar(ruidosa, ADC_BITS)


def a_i16(x):
    q = np.round(x * 32768.0)
    assert np.all(np.abs(q) < 32768)
    return q.astype("<i2")


def prueba_fft():
    rng = np.random.default_rng(1)
    x = rng.standard_normal(N).astype("<f4")
    f = TMP / "fft_in.f32"; x.tofile(f)
    subprocess.run([str(BIN), "fft", str(f), str(TMP / "fft_out.f32")], check=True)
    r = np.fromfile(TMP / "fft_out.f32", dtype="<f4")
    X = r[:2 * N:2] + 1j * r[1:2 * N:2]
    xr = r[2 * N::2]
    ref = np.fft.fft(x.astype(np.float64))
    err_rel = np.max(np.abs(X - ref)) / np.max(np.abs(ref))
    err_ifft = np.max(np.abs(xr - x))
    chequeo("P3 FFT vs numpy", err_rel < 1e-5, f"error relativo max = {err_rel:.2e}")
    chequeo("P3 IFFT(FFT(x)) = x", err_ifft < 1e-5, f"error abs max = {err_ifft:.2e}")
    return err_rel, err_ifft


def correr(cmd, x_i16, extra, salida):
    ent = TMP / "in.i16"; x_i16.tofile(ent)
    subprocess.run([str(BIN), cmd, str(ent), *extra], check=True)
    return np.fromfile(salida, dtype="<f4").astype(np.float64)


def prueba_bypass():
    rng = np.random.default_rng(5)
    x = a_i16(0.5 * np.clip(rng.standard_normal(L), -2, 2) / 2)
    # diag dummy: modo bypass (3er argumento = 1) ignora el piso
    diag = TMP / "dummy.bin"
    with open(diag, "wb") as f:
        for _ in range(N_BLOQUES):
            f.write(np.zeros(513, "<f4").tobytes() + np.uint16(0).tobytes())
    y = correr("ext", x, [str(diag), str(TMP / "by.f32"), "1"], TMP / "by.f32")
    # y tiene retardo HOP y 2 hops por bloque; arrastra HOP de cola no emitida
    xs = x.astype(np.float64) / 32768.0
    err = np.max(np.abs(y[HOP:HOP + L - HOP] - xs[:L - HOP]))
    chequeo("P4 bypass FFT+IFFT+OLA reconstruye", err < 2e-6, f"error abs max = {err:.2e} (escala +-1, float32)")
    salto = np.max(np.abs(np.diff(y[HOP:])))
    salto_ref = np.max(np.abs(np.diff(xs)))
    chequeo("P4 sin discontinuidades en las uniones", salto <= salto_ref * 1.001 + 1e-6,
            f"max |dy| = {salto:.4f} vs entrada {salto_ref:.4f}")
    return err


def volcar_diag(diags, ruta):
    with open(ruta, "wb") as f:
        for d in diags:
            f.write(d.piso.astype("<f4").tobytes() + np.uint16(len(d.interf)).tobytes()
                    + np.asarray(d.interf, "<u2").tobytes())


filas_aten = []


def prueba_escenario(pid, filas):
    esc, limpia, adc = senal_escenario(pid)
    x16 = a_i16(adc)
    # --- referencia Python (float64) con diagnostico del Nodo 1 (Python) ---
    nodo1, esp, diags, ref_out = NodoAdquisicion(esc["banda_util"]), NodoEspectral(), [], []
    xs = x16.astype(np.float64) / 32768.0                       # lo que realmente recibe el Nodo 3
    for i in range(N_BLOQUES):
        d = nodo1.procesar(xs[i * N:(i + 1) * N]); diags.append(d)
        for h in range(2):
            ref_out.append(esp.avanzar(xs[i * N + h * HOP: i * N + (h + 1) * HOP], d))
    ref_out = np.concatenate(ref_out)
    dfile = TMP / f"{pid}.diag"; volcar_diag(diags, dfile)

    # (a) solo el filtro, con el diagnostico de Python
    y_ext = correr("ext", x16, [str(dfile), str(TMP / "e.f32")], TMP / "e.f32")
    e_ext = np.max(np.abs(y_ext - ref_out))
    chequeo(f"P5 {pid} filtro C++ vs Python (mismo diag)", e_ext < 5e-4, f"error abs max = {e_ext:.2e}")

    # (b) estimador local C++ contra el del Nodo 1 en Python
    y_loc = correr("local", x16, [str(TMP / "l.f32"), str(esc["banda_util"][0]), str(esc["banda_util"][1]),
                                  str(TMP / "l.diag")], TMP / "l.f32")
    raw = (TMP / "l.diag").read_bytes(); pos, ig, ep, e_piso, e_abs = 0, 0, 0, 0.0, 0.0
    for d in diags:
        piso = np.frombuffer(raw, "<f4", 513, pos); pos += 513 * 4
        n = int(np.frombuffer(raw, "<u2", 1, pos)[0]); pos += 2
        bins = np.frombuffer(raw, "<u2", n, pos); pos += 2 * n
        # El redondeo de float32 pone un error ABSOLUTO ~1e-12 x pico de la PSD (~-117 dB; en P1 hay
        # bins 118 dB bajo el pico). Se exige: error relativo en bins > -60 dB del pico y error
        # absoluto respecto al pico en todos los bins.
        sig = d.piso > 1e-6 * d.psd.max()
        e_piso = max(e_piso, np.max(np.abs(piso - d.piso)[sig] / d.piso[sig], initial=0.0))
        e_abs = max(e_abs, np.max(np.abs(piso - d.piso)[~sig], initial=0.0) / d.psd.max())
        ig += int(sorted(bins.tolist()) == sorted(d.interf.tolist())); ep += 1
    chequeo(f"P5 {pid} piso local vs Nodo 1 (Python)", e_piso < 1e-3 and e_abs < 1e-10,
            f"error relativo max (bins > -60 dB) = {e_piso:.2e}; error absoluto/pico = {e_abs:.1e} ({10*np.log10(e_abs):.0f} dB)")
    chequeo(f"P5 {pid} interferentes locales == Nodo 1", ig == ep, f"{ig}/{ep} bloques con la misma lista de bins")
    e_loc = np.max(np.abs(y_loc - ref_out))
    chequeo(f"P5 {pid} salida (diag local) vs Python", e_loc < 5e-3, f"error abs max = {e_loc:.2e}")

    # (c) metricas del proyecto sobre la salida del C++ (retardo HOP ya descartado)
    sal_cpp = y_loc[HOP:HOP + L]; sal_py = ref_out[HOP:HOP + L]
    # cuantizacion de salida a 12 bits como en pipeline.py, para comparar con sus tablas
    q = signals.cuantizar
    m_cpp = metricas(limpia, adc, q(sal_cpp, 12), esc); m_py = metricas(limpia, adc, q(sal_py, 12), esc)
    filas.append((pid, m_py, m_cpp))

    # (d) clase Metrics del firmware (modo con referencia) contra metrics.metricas, ambos sin cuantizar la salida
    (TMP / "ref.f32").write_bytes(limpia.astype("<f4").tobytes())
    ent = TMP / "in_m.i16"; x16.tofile(ent)
    sal = subprocess.run([str(BIN), "metr", str(ent), str(TMP / "ref.f32"), *map(str, esc["banda_util"]),
                          *map(str, esc["rango_int"])], check=True, capture_output=True, text=True).stdout
    mc = {k: float(v) for k, v in (kv.split("=") for kv in sal.split())}
    mp = metricas(limpia, adc, sal_py, esc)
    ok = (abs(mc["snr_out"] - mp["snr_out"]) < 0.01 and abs(mc["snr_in"] - mp["snr_in"]) < 0.01
          and abs(mc["e_util"] - mp["e_util"]) < 0.05 and abs(mc["mse"] / mp["mse"] - 1) < 1e-3)
    chequeo(f"P5 {pid} Metrics (C++) == metricas() (Python)", ok,
            f"SNRout {mc['snr_out']:.3f}/{mp['snr_out']:.3f} dB, MSE {mc['mse']:.3e}/{mp['mse']:.3e}, Eutil {mc['e_util']:.2f}/{mp['e_util']:.2f} %")
    filas_aten.append((pid, mp["atenuacion"], mc["aten"], mc["util"], mc["snr_est_in"], mc["snr_est_out"], mp["snr_in"], mp["snr_out"]))
    chequeo(f"P5 {pid} dSNR C++ ~ Python", abs(m_cpp["dsnr"] - m_py["dsnr"]) < 0.2,
            f"dSNR py={m_py['dsnr']:.2f} dB, c++={m_cpp['dsnr']:.2f} dB")


if __name__ == "__main__":
    compilar()
    prueba_fft()
    prueba_bypass()
    filas = []
    for pid in sys.argv[1:] or ["P1", "P2", "P5"]:
        prueba_escenario(pid, filas)
    print("\nMetricas (Python float64 | C++ float32), ver tabla en el README:")
    print("ID  MSE(py)     MSE(c++)    SNRin   SNRout(py) SNRout(c++) dSNR(py) dSNR(c++) Eutil(py) Eutil(c++) Aten(py) Aten(c++)")
    for pid, a, b in filas:
        print(f"{pid}  {a['mse']:.3e}  {b['mse']:.3e}  {a['snr_in']:6.2f}  {a['snr_out']:8.2f}  {b['snr_out']:9.2f}  "
              f"{a['dsnr']:7.2f}  {b['dsnr']:8.2f}  {a['e_util']:8.1f}  {b['e_util']:8.1f}  {a['atenuacion']:6.1f}  {b['atenuacion']:6.1f}")
    print("\nSin referencia (estimaciones del firmware) frente a las verdaderas con referencia:")
    print("ID  Aten(py,senal completa)  Aten(fw,espectral)  UtilConserv(fw)  SNRin_est  SNRin_verd  SNRout_est  SNRout_verd")
    for pid, a_py, a_fw, u, si, so, vi, vo in filas_aten:
        print(f"{pid}  {a_py:20.1f} dB  {a_fw:15.1f} dB  {u:12.1f} %  {si:8.1f}   {vi:9.1f}   {so:9.1f}   {vo:10.1f}")
    print("\nRESULTADO:", "TODAS LAS COMPROBACIONES PASARON" if not fallos else f"FALLARON: {fallos}")
    sys.exit(1 if fallos else 0)
