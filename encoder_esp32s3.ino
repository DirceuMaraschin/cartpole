/*
  Encoder E38S6G5-600B-G24N
  ESP32 comum (ESP32-WROOM-32 / DevKit V1)

  LIGACOES:
  Vermelho -> 5 V
  Preto    -> GND
  Verde A  -> GPIO 18
  Branco B -> GPIO 19

  Um resistor de 4,6 kohm entre GPIO 18 e 3V3.
  Outro resistor de 4,6 kohm entre GPIO 19 e 3V3.
  Shield desconectado e isolado no teste inicial.

  Leitura x4: 2400 contagens por volta.
  Monitor Serial: 115200 baud.
  Envie z para zerar a posicao atual.
*/

#include <Arduino.h>

constexpr uint8_t PIN_A = 18;
constexpr uint8_t PIN_B = 19;

constexpr int SENTIDO = 1;  // Use -1 para inverter o sentido.
constexpr int32_t CONTAGENS_POR_VOLTA = 600 * 4;
constexpr uint32_t INTERVALO_MS = 100;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int64_t contagem = 0;
volatile uint8_t estadoAnterior = 0;

// Sequencia positiva: 00 -> 01 -> 11 -> 10 -> 00
DRAM_ATTR const int8_t transicao[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0
};

void IRAM_ATTR lerEncoder() {
  portENTER_CRITICAL_ISR(&mux);

  uint8_t atual =
      (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);

  contagem += SENTIDO * transicao[(estadoAnterior << 2) | atual];

  estadoAnterior = atual;

  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);

  // Utiliza os resistores externos ligados ao 3V3.
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);

  estadoAnterior =
      (digitalRead(PIN_A) << 1) | digitalRead(PIN_B);

  attachInterrupt(digitalPinToInterrupt(PIN_A), lerEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), lerEncoder, CHANGE);

  Serial.println("Encoder iniciado. Envie z para zerar.");
}

void loop() {
  static int64_t zero = 0;
  static uint32_t ultimaImpressao = 0;

  while (Serial.available() > 0) {
    char comando = Serial.read();

    if (comando == 'z' || comando == 'Z') {
      portENTER_CRITICAL(&mux);
      zero = contagem;
      portEXIT_CRITICAL(&mux);

      Serial.println("Zero definido na posicao atual.");
    }
  }

  uint32_t agora = millis();

  if (agora - ultimaImpressao >= INTERVALO_MS) {
    ultimaImpressao = agora;

    portENTER_CRITICAL(&mux);
    int64_t posicao = contagem - zero;
    portEXIT_CRITICAL(&mux);

    // Angulo acumulado: pode ultrapassar 360 graus.
    double angulo = posicao * (360.0 / CONTAGENS_POR_VOLTA);

    // Angulo dentro de uma volta: de 0 a menos de 360 graus.
    int64_t resto = posicao % CONTAGENS_POR_VOLTA;

    if (resto < 0) {
      resto += CONTAGENS_POR_VOLTA;
    }

    double anguloVolta = resto * (360.0 / CONTAGENS_POR_VOLTA);

    Serial.printf(
      "Contagem: %lld | Angulo: %.2f graus | 0-360: %.2f graus\n",
      (long long)posicao,
      angulo,
      anguloVolta
    );
  }
}