#pragma once
#include <cmath>

// Pasa altos de adquisicion (comando h<Hz>; defecto 100 Hz, h0 lo apaga).
// Butterworth de 2o orden (biquad RBJ, Q = 1/sqrt(2)), forma directa II transpuesta.
// Quita el DC y el ruido grave del microfono y su alimentacion (0-200 Hz, visto en capturas en
// silencio) ANTES de todo: el analisis, el piso de ruido que se envia y las muestras que recibe el
// nodo 3 salen de la misma senal, asi el nodo 3 no resta un piso inflado por graves que su senal no
// tiene. Ninguna prueba tiene senal util bajo 150 Hz: en 150 Hz atenua 0.8 dB, desde 300 Hz nada.
namespace nodo1 {

class PasaAltos {
public:
    void configurar(float fc_hz, float fs_hz) {
        fc_ = fc_hz;
        z1_ = z2_ = 0.0f;
        if (fc_hz <= 0.0f) return;
        const double w0 = 2.0 * M_PI * fc_hz / fs_hz, c = std::cos(w0);
        const double alfa = std::sin(w0) / (2.0 * M_SQRT1_2), a0 = 1.0 + alfa;
        b0_ = static_cast<float>((1.0 + c) / 2.0 / a0);
        b1_ = static_cast<float>(-(1.0 + c) / a0);
        a1_ = static_cast<float>(-2.0 * c / a0);
        a2_ = static_cast<float>((1.0 - alfa) / a0);
    }
    float procesar(float x) {
        if (fc_ <= 0.0f) return x;
        const float y = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * y + z2_;
        z2_ = b0_ * x - a2_ * y;               // b2 = b0
        return y;
    }
    float fc() const { return fc_; }

private:
    float fc_ = 0.0f, b0_ = 1.0f, b1_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f, z1_ = 0.0f, z2_ = 0.0f;
};

}  // namespace nodo1
