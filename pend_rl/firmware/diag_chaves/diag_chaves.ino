// ============================================================================
// Diagnostico das chaves de fim de curso e do encoder
// ============================================================================
//
// Nao move o motor. So le os pinos e mostra o que o firmware enxerga.
//
// Use quando o homing falhar com "FC A/B nao liberou apos recuo" ou
// "os dois fins de curso estao acionados".
//
// Monitor Serial em 115200.
//
// COMO LER:
//   Chave NF (normalmente fechada) ligada entre o pino e o GND, com pull-up
//   interno. Entao:
//      chave LIVRE  (nao pressionada) -> contato FECHADO -> pino em LOW  -> "livre"
//      chave PRESSIONADA              -> contato ABERTO  -> pino em HIGH -> "ACIONADA"
//      FIO ROMPIDO ou GND solto       -> pino flutua     -> HIGH         -> "ACIONADA"
//
//   O ultimo caso e proposital: a falha do cabo e lida como chave acionada,
//   e o sistema para em vez de atropelar o limite.
//
// O QUE PROCURAR:
//   - Com as duas chaves livres, as duas linhas devem dizer "livre".
//   - Se uma disser ACIONADA sem ninguem tocar nela, siga o checklist no fim
//     deste arquivo.
//   - Pressione cada chave com o dedo e confirme que ela muda de estado.
// ============================================================================

#include <Arduino.h>

const int PIN_FC_A  = 32;
const int PIN_FC_B  = 33;
const int PIN_ENC_A = 18;
const int PIN_ENC_B = 19;
const int ACIONADO  = HIGH;

volatile long contagemEnc = 0;
volatile uint8_t estadoEnc = 0;
const int8_t tabelaEnc[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR lerEncoder() {
  uint8_t e = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  contagemEnc += tabelaEnc[(estadoEnc << 2) | e];
  estadoEnc = e;
}

// Mesma leitura com debounce do firmware principal.
bool fimDeCurso(int pino) {
  if (digitalRead(pino) != ACIONADO) return false;
  delayMicroseconds(200);
  if (digitalRead(pino) != ACIONADO) return false;
  delayMicroseconds(200);
  return digitalRead(pino) == ACIONADO;
}

// Conta trocas de estado em 1 s: revela mau contato intermitente.
volatile uint32_t trocasA = 0, trocasB = 0;
void IRAM_ATTR isrA() { trocasA++; }
void IRAM_ATTR isrB() { trocasB++; }

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_FC_A, INPUT_PULLUP);
  pinMode(PIN_FC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  estadoEnc = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), lerEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), lerEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_FC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_FC_B), isrB, CHANGE);

  Serial.println();
  Serial.println("=== DIAGNOSTICO DAS CHAVES E DO ENCODER ===");
  Serial.println("O motor nao sera acionado.");
  Serial.println();
  Serial.println("Esperado com as duas chaves LIVRES: as duas em 'livre'.");
  Serial.println("Pressione cada chave com o dedo e veja se muda para ACIONADA.");
  Serial.println("Gire a haste com a mao e veja o angulo mudar.");
  Serial.println();
  Serial.println("tempo |  FC_A (32)       |  FC_B (33)       | trocas/s | encoder");
  Serial.println("------+------------------+------------------+----------+---------");
}

void loop() {
  static uint32_t t0 = 0;
  if (millis() - t0 < 500) return;
  t0 = millis();

  static uint32_t tA = 0, tB = 0;
  uint32_t dA = trocasA - tA; tA = trocasA;
  uint32_t dB = trocasB - tB; tB = trocasB;

  int bruA = digitalRead(PIN_FC_A);
  int bruB = digitalRead(PIN_FC_B);
  bool fcA = fimDeCurso(PIN_FC_A);
  bool fcB = fimDeCurso(PIN_FC_B);

  long enc = contagemEnc;
  float graus = enc * (360.0f / 2400.0f);

  char linha[150];
  snprintf(linha, sizeof(linha),
           "%5lu | %s %-12s | %s %-12s | %3lu %3lu  | %+8.1f deg",
           (unsigned long)(millis() / 1000),
           bruA ? "HIGH" : "LOW ", fcA ? "ACIONADA" : "livre",
           bruB ? "HIGH" : "LOW ", fcB ? "ACIONADA" : "livre",
           (unsigned long)dA, (unsigned long)dB, graus);
  Serial.println(linha);

  // Veredito imediato quando algo esta errado em repouso.
  static int avisos = 0;
  if (avisos < 3 && (fcA || fcB) && dA == 0 && dB == 0) {
    avisos++;
    Serial.println();
    Serial.print("  >> ");
    if (fcA && fcB) Serial.println("AS DUAS chaves lidas como acionadas.");
    else if (fcA)   Serial.println("FC_A (pino 32) lido como ACIONADO.");
    else            Serial.println("FC_B (pino 33) lido como ACIONADO.");
    Serial.println("  >> Se a chave esta fisicamente livre, o circuito esta aberto.");
    Serial.println("  >> Checklist, nesta ordem:");
    Serial.println("     1. O fio do GND da chave esta firme? (causa mais comum)");
    Serial.println("     2. O conector/borne esta solto ou oxidado?");
    Serial.println("     3. A chave esta ligada em NF (terminais COM e NC)?");
    Serial.println("        Em NA (COM e NO) a leitura fica invertida.");
    Serial.println("     4. Teste: ligue um jumper do pino direto ao GND.");
    Serial.println("        Se virar 'livre', o problema esta no cabo/chave.");
    Serial.println("        Se continuar ACIONADA, o problema esta no pino da placa.");
    Serial.println();
  }
  if (avisos < 6 && (dA > 4 || dB > 4)) {
    avisos++;
    Serial.println("  >> MUITAS trocas de estado: mau contato intermitente ou ruido.");
    Serial.println("     Afaste os fios da chave dos fios do motor.");
  }
}
