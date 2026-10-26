// ESP32 - Pêndulo invertido sobre carrinho com motor de passo (STEP/DIR, microstep 1/8)
// Homing + centro + swing-up + PID de equilibrio em 180 graus
//
// Convenção de ângulo: 0°/360° = pêndulo pendurado (pra baixo), 180° = em pé.
// Convenção de lado: o sentido A -> B do carrinho é o "positivo".
//
// Ligações:
//   STEP -> 25 | DIR -> 26 | EN -> 27
//   Fim de curso A -> 32 e GND | Fim de curso B -> 33 e GND (NF, pull-up interno)
//   Encoder A -> 18 | Encoder B -> 19 (pull-up externo 4.7k para 3.3V)
//
// Comandos (Monitor Serial 115200):
//   i = centralizar e iniciar swing-up + PID    s = parar
//   e = centralizar e aguardar elevacao MANUAL para testar apenas o PID
// Ganhos iniciais, NAO sintonizados na bancada. Posicao estimada por passos.
// Pinagem mantida do programa recebido: ESP32 classico, nao ESP32-S3.
// Primeiro teste: KP/KD, com KI=0; depois reative KI aos poucos.
// Perto do topo, inclinacao para B deve produzir movimento para B.
// Se ocorrer o contrario, verifique ENC_SINAL e DIR_PARA_A/B.
//   a / b / c = ir ao lado A / lado B / centro
//   z = definir 0° (pêndulo pendurado e parado)    p = mostrar posição e ângulo

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <ctype.h>
#include <math.h>

// ===================== PINOS =====================
const int PIN_STEP  = 25;
const int PIN_DIR   = 26;
const int PIN_EN    = 27;
const int PIN_FC_A  = 32;
const int PIN_FC_B  = 33;
const int PIN_ENC_A = 18;
const int PIN_ENC_B = 19;

// ===================== CONFIGURAÇÃO BÁSICA =====================
const int ACIONADO   = HIGH;   // fim de curso NF: acionado = HIGH
const int DIR_PARA_A = LOW;
const int DIR_PARA_B = HIGH;

const int  PASSOS_POR_VOLTA    = 200 * 8;  // 1600
const long INTERVALO_HOMING_US = 800;
const long INTERVALO_MEDIDA_US = 500;
const long RECUO_PASSOS        = 80;
const long LIMITE_PASSOS       = 200000;

// Encoder
const int CONTAGENS_POR_VOLTA = 600 * 4;   // 2400 (x4)
const int ENC_SINAL = -1;   // inverta para -1 se o ângulo cresce para o lado errado (veja o teste)

// ===================== MECÂNICA (AJUSTE!) =====================
// Distância real, em metros, entre os pontos onde os dois fins de curso acionam
// (meça com régua). Com 0 usa o valor padrão abaixo.
const float CURSO_REAL_M = 0.0;
// Padrão: polia GT2 de 20 dentes (40 mm por volta) com 1600 passos/volta
const float PASSOS_POR_M_PADRAO = 40000.0;
// Comprimento equivalente do pêndulo, em metros. Haste uniforme: 2/3 do comprimento.
// Massa concentrada na ponta: distância do eixo até a massa.
const float L_EQ_M = 0.220;

// ===================== LIMITES DE MOVIMENTO =====================
const float V_MAX_PASSOS = 18000.0;  // passos/s (no máximo 18000)
const float A_IR_MS2     = 1.0;     // aceleração dos comandos a/b/c (m/s²)
const float V_IR_MS      = 0.15;    // velocidade dos comandos a/b/c (m/s)
const float A_FREIO_MS2  = 8.0;     // freada usada perto das extremidades (m/s²)
const float MARGEM_M     = 0.01;    // folga das extremidades no swing-up/equilíbrio

// ===================== SWING-UP =====================
const float K_SW     = 6.0;   // ganho da energia
const float A_SW_MAX = 4.0;   // aceleração máxima do swing-up (m/s²)
const float KX_SW    = 1.0;  // mola virtual para manter o carrinho perto do centro
const float KV_SW    = 0.1;

// ===================== EQUILÍBRIO =====================
const float CAPTURA_GRAUS = 20.0;  // começa a equilibrar quando estiver a menos disso de 180°
const float CAPTURA_VEL   = 5.0;   // rad/s máximo para capturar
const float SAIDA_GRAUS   = 35.0;  // se passar disso, volta ao swing-up
const float KP = 40;   // m/s² por rad
const float KI = 1;    // m/s^2 por (rad.s), valor inicial para ajuste
const float I_MAX = 0.5; // contribuicao integral maxima em m/s^2
const float KD = 5.5;    // m/s² por rad/s
const float KX = 1.5;    // posição do carrinho
const float KV = 17.5;    // velocidade do carrinho
const float A_BAL_MAX = 8.0;

// ===================== VARIÁVEIS GLOBAIS =====================
void parar(const char* motivo);

volatile long posicao = 0;           // passos relativos ao zero (lado A)
long limiteB = 0;                    // posição máxima segura
float passosPorM = PASSOS_POR_M_PADRAO;
volatile long limiteInfA = -1000000000L;
volatile long limiteSupB =  1000000000L;

const float W0 = sqrtf(9.81f / L_EQ_M);

// ---------- Encoder ----------
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

long lerContagemEnc() {
  noInterrupts();
  long c = contagemEnc;
  interrupts();
  return c;
}

void zerarEncoder() {
  noInterrupts();
  contagemEnc = 0;
  interrupts();
}

// ---------- Gerador de passos por timer (só inteiros dentro da ISR) ----------
const uint32_t TICK_HZ = 40000;      // 25 us por tick
volatile uint32_t incFase = 0;       // Q16: passos por tick
volatile int8_t dirCmd = 0;          // +1 = para B, -1 = para A, 0 = parado
hw_timer_t* timerPassos = nullptr;

void IRAM_ATTR onTimer() {
  static uint32_t fase = 0;
  static bool pulsoAlto = false;
  static int dirAtual = -1;

  bool acabouPulso = pulsoAlto;
  if (pulsoAlto) {
    digitalWrite(PIN_STEP, LOW);
    pulsoAlto = false;
  }
  int8_t d = dirCmd;
  // Bloqueia novos pulsos imediatamente ao abrir qualquer NF.
  if (d == 0 || digitalRead(PIN_FC_A) == ACIONADO ||
      digitalRead(PIN_FC_B) == ACIONADO) { fase = 0; return; }
  int sentido = (d > 0) ? DIR_PARA_B : DIR_PARA_A;
  if (sentido != dirAtual) {
    digitalWrite(PIN_DIR, sentido);
    dirAtual = sentido;
    fase = 0;
    return; // tempo de preparacao do DIR
  }
  if ((d > 0 && posicao >= limiteSupB) ||
      (d < 0 && posicao <= limiteInfA)) { fase = 0; return; }

  // Acumula TAMBEM no tick que encerra o pulso, evitando erro de frequencia.
  fase += incFase;
  if (!acabouPulso && fase >= 65536) {
    fase -= 65536;
    digitalWrite(PIN_STEP, HIGH);
    pulsoAlto = true;
    posicao += d;
  }
}

void iniciarTimer() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  timerPassos = timerBegin(1000000);
  if (!timerPassos) parar("falha ao iniciar timer");
  timerAttachInterrupt(timerPassos, &onTimer);
  timerAlarm(timerPassos, 1000000 / TICK_HZ, true, 0);
#else
  timerPassos = timerBegin(0, 80, true);
  if (!timerPassos) parar("falha ao iniciar timer");
  timerAttachInterrupt(timerPassos, &onTimer, true);
  timerAlarmWrite(timerPassos, 1000000 / TICK_HZ, true);
  timerAlarmEnable(timerPassos);
#endif
}

void aplicarVelocidade(float v_ms) {
  float vp = v_ms * passosPorM;
  if (vp > V_MAX_PASSOS) vp = V_MAX_PASSOS;
  if (vp < -V_MAX_PASSOS) vp = -V_MAX_PASSOS;
  float a = fabsf(vp);
  if (a < 1.0f) {
    dirCmd = 0;
    incFase = 0;
    return;
  }
  incFase = (uint32_t)(a * 65536.0f / TICK_HZ);
  dirCmd = (vp > 0) ? 1 : -1;
}

// ---------- Homing e medição (bloqueantes, antes de ligar o timer) ----------
bool fimDeCurso(int pino) {
  if (digitalRead(pino) != ACIONADO) return false;
  delayMicroseconds(200);
  if (digitalRead(pino) != ACIONADO) return false;
  delayMicroseconds(200);
  return digitalRead(pino) == ACIONADO;
}

void darPasso(int sentido, long intervaloUs) {
  digitalWrite(PIN_DIR, sentido);
  delayMicroseconds(5);
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(intervaloUs);
  posicao += (sentido == DIR_PARA_B) ? 1 : -1;
}

long andarAteFimDeCurso(int pinoFC, int sentido, long intervaloUs) {
  long passos = 0;
  while (!fimDeCurso(pinoFC)) {
    darPasso(sentido, intervaloUs);
    if (++passos > LIMITE_PASSOS) return -1;
  }
  return passos;
}

void andarPassos(long n, int sentido, long intervaloUs) {
  for (long i = 0; i < n; i++) darPasso(sentido, intervaloUs);
}

void parar(const char* motivo) {
  dirCmd = 0;
  incFase = 0;
  digitalWrite(PIN_EN, HIGH);
  Serial.print("ERRO: ");
  Serial.println(motivo);
  while (true) delay(1000);
}

// ---------- Estimativa de ângulo e velocidade angular ----------
const int N_HIST = 8;
float phiHist[N_HIST];
uint32_t tHist[N_HIST];
int idxHist = 0;
float phi = 0;     // rad, 0 = pendurado, cresce para o lado B
float omega = 0;   // rad/s

float anguloRad() {
  return ENC_SINAL * (float)lerContagemEnc() * (2.0f * PI / CONTAGENS_POR_VOLTA);
}

void iniciarHistorico() {
  phi = anguloRad();
  idxHist = 0;
  uint32_t t = micros();
  for (int i = 0; i < N_HIST; i++) { phiHist[i] = phi; tHist[i] = t; }
  omega = 0;
}

void atualizarAngulo() {
  phi = anguloRad();
  uint32_t t = micros();
  float janela = (uint32_t)(t - tHist[idxHist]) * 1e-6f;
  if (janela > 0.0005f) {
    float bruto = (phi - phiHist[idxHist]) / janela;
    omega = 0.5f * omega + 0.5f * bruto;
  }
  phiHist[idxHist] = phi;
  tHist[idxHist] = t;
  idxHist = (idxHist + 1) % N_HIST;
}

float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

float anguloGraus() {
  float d = fmodf(phi * 180.0f / PI, 360.0f);
  if (d < 0) d += 360.0f;
  return d;
}

static inline float limitar(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ---------- Controle ----------
enum Modo { PARADO, IR_PARA, AGUARDAR_TOPO, SWINGUP, EQUILIBRIO };
Modo modo = PARADO;
Modo aposCentro = PARADO;
bool usarSwingup = true;
float termoI = 0; // KI * integral(psi dt), em m/s^2
uint32_t chegouCentroEm = 0;

float vMs = 0;            // velocidade comandada do carrinho (m/s)
long alvoPassos = 0;
bool zeroDefinido = false;
uint32_t tCtl = 0;

void rampaPara(float vDes, float aMax, float dt) {
  float dv = vDes - vMs;
  float passo = aMax * dt;
  if (dv > passo) dv = passo;
  else if (dv < -passo) dv = -passo;
  vMs += dv;
}

void mostrarPosicao() {
  float x_mm = (posicao - limiteB / 2) * 1000.0f / passosPorM;
  Serial.printf("Posicao: %ld passos (%.0f mm do centro) | Angulo: %.1f graus\n",
                (long)posicao, x_mm, anguloGraus());
}

void controle(float dt) {
  atualizarAngulo();

  float x = (posicao - limiteB / 2) / passosPorM;   // metros, relativo ao centro
  float vMax = V_MAX_PASSOS / passosPorM;
  float a = 0;
  float integralAnterior = termoI;
  bool calculouPID = false;
  bool saturou = false;

  // Segurança: fim de curso físico acionado (ou fio solto, já que é NF)
  if (fimDeCurso(PIN_FC_A) || fimDeCurso(PIN_FC_B)) {
    if (modo != PARADO) Serial.println("Fim de curso acionado! Parando.");
    modo = PARADO;
    aposCentro = PARADO;
    termoI = 0;
    vMs = 0;
    aplicarVelocidade(0);
    return;
  }

  switch (modo) {
    case PARADO:
      rampaPara(0, A_FREIO_MS2, dt);
      break;

    case IR_PARA: {
      float dPassos = (float)(alvoPassos - posicao);
      float d = dPassos / passosPorM;
      float vDes = 0;
      if (fabsf(dPassos) > 2) {
        vDes = sqrtf(2.0f * 0.8f * A_IR_MS2 * fabsf(d));
        if (vDes > V_IR_MS) vDes = V_IR_MS;
        if (d < 0) vDes = -vDes;
      }
      rampaPara(vDes, A_IR_MS2, dt);
      if (fabsf(dPassos) <= 2 && fabsf(vMs) < 0.005f) {
        modo = aposCentro;
        aposCentro = PARADO;
        chegouCentroEm = millis();
        termoI = 0;
        vMs = 0;
        Serial.println("Chegou.");
        mostrarPosicao();
      }
      break;
    }

    case AGUARDAR_TOPO: {
      vMs = 0;
      float psi = wrapPi(PI - phi);
      if (usarSwingup) {
        // Repouso no meio antes de iniciar a elevacao automatica.
        if (millis() - chegouCentroEm >= 2000) modo = SWINGUP;
      } else if (fabsf(psi) < CAPTURA_GRAUS * PI / 180.0f &&
                 fabsf(omega) < CAPTURA_VEL) {
        termoI = 0;
        modo = EQUILIBRIO;
        Serial.println("PID capturou o pendulo.");
      }
      break;
    }

    case SWINGUP: {
      float psi = wrapPi(PI - phi);   // inclinação em relação ao topo (+ = lado B)
      float psiDot = -omega;

      // Energia normalizada: parado embaixo = -1, topo = +1.
      float En = 0.5f * (omega / W0) * (omega / W0) - cosf(phi);
      float dEn = 1.0f - En;
      float s = omega * cosf(phi);
      float sg = (fabsf(omega) < 0.2f) ? 1.0f : ((s > 0) ? 1.0f : -1.0f);
      float u = limitar(K_SW * dEn, -A_SW_MAX, A_SW_MAX);
      a = -u * sg - KX_SW * x - KV_SW * vMs;

      if (fabsf(psi) < CAPTURA_GRAUS * PI / 180.0f && fabsf(psiDot) < CAPTURA_VEL) {
        modo = EQUILIBRIO;
        termoI = 0;
        Serial.println("Capturado! Equilibrando...");
      }
      break;
    }

    case EQUILIBRIO: {
      float psi = wrapPi(PI - phi);
      float psiDot = -omega;
      if (fabsf(psi) > SAIDA_GRAUS * PI / 180.0f) {
        termoI = 0;
        // Swing-up so e repetido se escolhido pelo comando i.
        modo = usarSwingup ? SWINGUP : PARADO;
        if (!usarSwingup) { vMs = 0; aplicarVelocidade(0); }
        Serial.println("Perdeu equilibrio: swing-up ou parada conforme comando.");
        break;
      }
      // PID angular + realimentacao de posicao/velocidade original.
      // psi positivo = inclinacao para B perto do topo; a positivo = B.
      float base = KP * psi + KD * psiDot + KX * x + KV * vMs;
      float candidato = (KI > 0) ? limitar(termoI + KI * psi * dt, -I_MAX, I_MAX) : 0;
      float bruto = base + candidato;
      saturou = fabsf(bruto) > A_BAL_MAX;
      // Integracao condicional: permite descarregar a integral na saturacao.
      if (!saturou || (bruto > A_BAL_MAX && psi < 0) ||
                     (bruto < -A_BAL_MAX && psi > 0)) termoI = candidato;
      a = limitar(base + termoI, -A_BAL_MAX, A_BAL_MAX);
      calculouPID = true;
      break;
    }
  }

  if (modo == SWINGUP || modo == EQUILIBRIO) {
    float vBruta = vMs + a * dt;
    vMs = limitar(vBruta, -vMax, vMax);
    bool limitouVelocidade = (vMs != vBruta);

    // Freio de parede: nunca vai mais rápido do que consegue frear antes da extremidade
    float dB = (limiteB - posicao) / passosPorM - MARGEM_M;
    float dA = posicao / passosPorM - MARGEM_M;
    if (dB < 0) dB = 0;
    if (dA < 0) dA = 0;
    float vB = sqrtf(2.0f * A_FREIO_MS2 * dB);
    float vA = sqrtf(2.0f * A_FREIO_MS2 * dA);
    if (vMs > vB) { vMs = vB; limitouVelocidade = true; }
    if (vMs < -vA) { vMs = -vA; limitouVelocidade = true; }
    // Nao acumula integral quando velocidade/curso limitam o atuador.
    if (calculouPID && limitouVelocidade) termoI = integralAnterior;
  }

  vMs = limitar(vMs, -vMax, vMax);
  aplicarVelocidade(vMs);
}

void tarefaControle() {
  uint32_t agora = micros();
  if ((uint32_t)(agora - tCtl) < 1000) return;   // 1 kHz
  float dt = (uint32_t)(agora - tCtl) * 1e-6f;
  tCtl = agora;
  if (dt > 0.010f) {
    modo = PARADO;
    aposCentro = PARADO;
    termoI = 0;
    vMs = 0;
    aplicarVelocidade(0);
    Serial.println("Controle atrasou mais de 10 ms. Parado; reinicie com i/e.");
    return;
  }
  controle(dt);
}

void tratarComando(char c) {
  switch (tolower(c)) {
    case 'i':
    case 'e':
      if (!zeroDefinido) { Serial.println("Pendulo para baixo e parado: envie z primeiro."); break; }
      if (modo != PARADO || fabsf(vMs) > 0.001f) {
        Serial.println("Pare com s e espere antes de iniciar."); break;
      }
      iniciarHistorico();
      termoI = 0;
      usarSwingup = (tolower(c) == 'i');
      alvoPassos = limiteB / 2;
      aposCentro = AGUARDAR_TOPO;
      modo = IR_PARA;
      Serial.println(usarSwingup ? "Centralizando para swing-up + PID." :
        "Centralizando. Ao chegar, eleve o pendulo; PID captura perto de 180 graus.");
      break;
    case 's':
      modo = PARADO;
      aposCentro = PARADO;
      termoI = 0;
      Serial.println("Parando.");
      break;
    case 'a': case 'b': case 'c':
      termoI = 0;
      aposCentro = PARADO;
      alvoPassos = tolower(c) == 'a' ? 0 : (tolower(c) == 'b' ? limiteB : limiteB / 2);
      modo = IR_PARA;
      break;
    case 'z':
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("Pare antes de zerar (comando s)."); break; }
      zerarEncoder();
      iniciarHistorico();
      zeroDefinido = true;
      Serial.println("Zero definido: pendulo pendurado = 0 graus.");
      break;
    case 'p': mostrarPosicao(); break;
    case '\n': case '\r': case ' ': break;
    default:
      Serial.println("Comandos: i (swing-up+PID), e (PID manual), s (parar), a/b/c (mover), z (zero), p (posicao)");
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FC_A, INPUT_PULLUP);
  pinMode(PIN_FC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  estadoEnc = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), lerEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), lerEncoder, CHANGE);

  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, LOW);
  delay(500);

  if (fimDeCurso(PIN_FC_A) && fimDeCurso(PIN_FC_B)) {
    parar("os dois fins de curso estao acionados (fio solto ou logica errada)");
  }

  Serial.println("Procurando fim de curso A...");
  if (!fimDeCurso(PIN_FC_A)) {
    if (andarAteFimDeCurso(PIN_FC_A, DIR_PARA_A, INTERVALO_HOMING_US) < 0)
      parar("fim de curso A nao encontrado");
  }
  andarPassos(RECUO_PASSOS, DIR_PARA_B, INTERVALO_HOMING_US);
  if (fimDeCurso(PIN_FC_A)) parar("FC A nao liberou apos recuo");
  posicao = 0;

  Serial.println("Medindo ate o fim de curso B...");
  if (andarAteFimDeCurso(PIN_FC_B, DIR_PARA_B, INTERVALO_MEDIDA_US) < 0)
    parar("fim de curso B nao encontrado");
  long distanciaPassos = posicao;
  andarPassos(RECUO_PASSOS, DIR_PARA_A, INTERVALO_HOMING_US);
  if (fimDeCurso(PIN_FC_B)) parar("FC B nao liberou apos recuo");
  limiteB = posicao;
  if (limiteB <= 2 * RECUO_PASSOS) parar("curso medido invalido");

  if (CURSO_REAL_M > 0) passosPorM = (distanciaPassos + RECUO_PASSOS) / CURSO_REAL_M;

  Serial.printf("Curso: %ld passos | %.0f passos/m (%s) | curso util %.0f mm\n",
                distanciaPassos, passosPorM,
                CURSO_REAL_M > 0 ? "medido" : "padrao - ajuste CURSO_REAL_M",
                1000.0f * limiteB / passosPorM);

  limiteInfA = 0;
  limiteSupB = limiteB;

  iniciarHistorico();
  iniciarTimer();
  tCtl = micros();

  // Centralizacao nao bloqueante; comandos continuam disponiveis no loop.
  alvoPassos = limiteB / 2;
  modo = IR_PARA;
  zeroDefinido = false;
  Serial.println("Indo ao meio. Depois de chegar e o pendulo parar para baixo, envie z.");
  Serial.println("i = centralizar + swing-up + PID | e = centralizar + captura manual | s = parar");

}

// ===================== LOOP =====================
void loop() {
  if (Serial.available()) tratarComando(Serial.read());
  tarefaControle();

  static uint32_t tPrint = 0;
  if ((modo == SWINGUP || modo == EQUILIBRIO) && millis() - tPrint > 200) {
    tPrint = millis();
    float x_mm = (posicao - limiteB / 2) * 1000.0f / passosPorM;
    char linha[110];
    int n = snprintf(linha, sizeof(linha), "%s ang=%.1f x=%.0fmm v=%.3f I=%.3f\n",
      modo == SWINGUP ? "SWING" : "PID", anguloGraus(), x_mm, vMs, termoI);
    if (n > 0 && n < (int)sizeof(linha) && Serial.availableForWrite() >= n)
      Serial.write((const uint8_t*)linha, n);
  }
}