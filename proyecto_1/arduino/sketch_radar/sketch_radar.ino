const int PIN_BUZZER = 9;
const int PIN_MIC = A0;

const int NUM_MUESTRAS = 600;      // 60 ms de grabación total a 10 kHz
const int FREC_TONO = 2000;
const unsigned long PERIODO_US = 100;
const unsigned long DURACION_PULSO_US = 10000; // 10 ms de tono para resonancia real

uint16_t muestras[NUM_MUESTRAS];

void setup() {
  // analogReference(INTERNAL);
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  ADCSRA = (ADCSRA & 0b11111000) | 0b00000110; // ADC acelerado a 10 kHz
}

void loop() {
  if (Serial.available() > 0) {
    char comando = Serial.read();
    if (comando == 'M' || comando == 'm') {

      unsigned long t_inicio = micros();
      unsigned long siguiente_muestra = t_inicio;

      tone(PIN_BUZZER, FREC_TONO);

      for (int i = 0; i < NUM_MUESTRAS; i++) {
        while ((long)(micros() - siguiente_muestra) < 0) {}

        muestras[i] = analogRead(PIN_MIC);

        // Apaga el buzzer tras 10 ms (muestra 100)
        if ((unsigned long)(micros() - t_inicio) >= DURACION_PULSO_US) {
          noTone(PIN_BUZZER);
          digitalWrite(PIN_BUZZER, LOW);
        }

        siguiente_muestra += PERIODO_US;
      }

      noTone(PIN_BUZZER);
      digitalWrite(PIN_BUZZER, LOW);

      for (int i = 0; i < NUM_MUESTRAS; i++) {
        Serial.println(muestras[i]);
      }
    }
  }
}
