// pnmpc_kan_bancada.ino - PNMPC-KAN NA BANCADA REAL (ESP32 clássico)
// Pêndulo sobre carrinho com motor de passo (STEP/DIR, microstep 1/8): swing-up + equilíbrio em pé.
//
// Hardware e convenções IGUAIS ao firmware pid_IA (Planta_smiluação/pid_IA.cpp): mesmo gerador de
// passos por timer, encoder em quadratura, homing, estimador de ângulo/omega, freio de parede e
// parada nos fins de curso. Só o controlador muda: o swing-up + PID dá lugar ao PNMPC-KAN
// (pnmpc_kan.h), com a KAN de volta inteira (kan_planta_pesos.h, gerado por ident_preparar_volta.m).
//
// Núcleos:
//   núcleo 1: loop() a 1 kHz (ângulo, integração a -> v, segurança, passos) + ISRs do timer e do encoder
//   núcleo 0: tarefa "PNMPC" a cada 10 ms (KAN_TS), dedicada ao PNMPC-KAN, com compensação do
//             atraso de cálculo (prevê o estado em k+1 com a KAN e o a_cmd resultante entra em k+1)
//
// Convenções (as do pid_IA e do ensaio 154800): ângulo 0 = pendurado, 180 = em pé (no SENSOR do
// MATLAB: 270 = pendurado, 90 = em pé); sentido A -> B do carrinho = positivo; a > 0 empurra para B.
//
// Ligações (ESP32 clássico):
//   STEP -> 25 | DIR -> 26 | EN -> 27
//   Fim de curso A -> 32 e GND | Fim de curso B -> 33 e GND (NF, pull-up interno)
//   Encoder A -> 18 | Encoder B -> 19 (pull-up externo 4.7k para 3.3V)
//
// MODO AUTÔNOMO (INICIO_AUTOMATICO = true): ao ligar faz o homing, vai ao centro, espera o pêndulo
// pendurado e parado, define o zero e liga o PNMPC-KAN sozinho; após parada de segurança recentraliza
// e tenta de novo (até MAX_TENTATIVAS). A serial é opcional (só telemetria e comandos manuais).
//
// Comandos (Monitor Serial 115200, um caractere, opcionais):
//   z = definir 0 (pêndulo pendurado e PARADO)       i = centralizar e iniciar o PNMPC-KAN (swing-up + topo)
//   s = parar                                        a / b / c = ir ao lado A / lado B / centro
//   p = mostrar posição e ângulo                     l = liga/desliga o log CSV a 100 Hz
//   t = mede o tempo do PNMPC (motor parado)         k = teste de sinal: empurrão de ~1 cm, confere a convenção
//
// SEGURANÇA: teste primeiro com o pêndulo SEM a haste (ou segurando-a), confira o sentido do motor
// (a > 0 deve mover para B) e do encoder (ENC_SINAL), e mantenha a mão no botão de energia do driver.

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <ctype.h>
#include <math.h>
// Horizonte do PNMPC nesta placa: na ESP32 Dev Module (LX6) o PNMPC com N = 40 levou 8,6 ms
// (máx 9,8 ms) para um período de 10 ms. Com N = 30 sobra folga; o custo terminal LQR cobre o resto.
#define MPC_N 30
#include "kan_planta_pesos.h"
#include "kan_eval.h"
#include "pnmpc_kan.h"

#if CONFIG_FREERTOS_UNICORE
  #error "O PNMPC-KAN precisa de um ESP32 de dois núcleos."
#endif

// ===================== PINOS (ESP32 clássico) =====================
const int PIN_STEP  = 25;
const int PIN_DIR   = 26;
const int PIN_EN    = 27;
const int PIN_FC_A  = 32;
const int PIN_FC_B  = 33;
const int PIN_ENC_A = 18;
const int PIN_ENC_B = 19;

// ===================== CONFIGURAÇÃO BÁSICA (igual ao pid_IA) =====================
const int ACIONADO   = HIGH;   // fim de curso NF: acionado = HIGH
const int DIR_PARA_A = LOW;
const int DIR_PARA_B = HIGH;

const long INTERVALO_HOMING_US = 800;
const long INTERVALO_MEDIDA_US = 500;
const long RECUO_PASSOS        = 400;   // 10 mm: com 80 (2 mm) a chave B continuava lida como acionada
const long LIMITE_PASSOS       = 200000;

const int CONTAGENS_POR_VOLTA = 600 * 4;   // 2400 (x4)
const int ENC_SINAL = -1;

const float CURSO_REAL_M = 0.0;            // meça e ajuste (0 = usa PASSOS_POR_M_PADRAO)
const float PASSOS_POR_M_PADRAO = 40000.0; // polia GT2 20 dentes, 1600 passos/volta

const float V_MAX_PASSOS = 18000.0;  // passos/s
const float A_IR_MS2     = 1.0;      // comandos a/b/c
const float V_IR_MS      = 0.15;
const float A_FREIO_MS2  = 8.0;      // freio de parede
const float MARGEM_M     = 0.01;

// ===================== PNMPC-KAN =====================
const float TH_REF        = PI;      // referência de ângulo: em pé (= 90 graus no sensor do MATLAB)
const float X_REF         = 0.0f;    // referência de posição (centro) [m]
const float FOLGA_SOFT_M  = 0.08f;   // a penalidade de trilho do PNMPC começa a (meio curso - isto)
const uint32_t PNMPC_TIMEOUT_MS = 40; // sem resposta nova do PNMPC por mais que isto -> PARADO

// ===================== MODO AUTÔNOMO (sem serial) =====================
// Ao ligar: homing -> centro -> espera o pêndulo pendurado e PARADO -> zero automático -> PNMPC-KAN.
// Depois de uma parada de segurança: volta ao centro, espera parar e tenta de novo (até MAX_TENTATIVAS).
const bool  INICIO_AUTOMATICO = true;
const int   MAX_TENTATIVAS    = 3;
const float OMEGA_REPOUSO     = 0.10f;    // rad/s: abaixo disso o pêndulo é considerado parado
const uint32_t T_REPOUSO_MS   = 2000;     // tempo parado antes de zerar/ligar

// Observador de theta/omega com a KAN (núcleo 0): ganhos testados no simulador com o ruído de
// omega medido na bancada (omega_med = omega + 0,4*a + ruído).
const float OBS_L_TH = 0.6f;              // correção do ângulo pela inovação do ângulo
const float OBS_L_W  = 10.0f;             // correção de omega [1/s] pela inovação do ângulo
float so[4] = {0, 0, 0, 0};               // estado estimado (usado pelo PNMPC)
bool  obsOk = false;
float aAnterior = 0.0f;                   // a_cmd aplicado no período anterior

// ===================== VARIÁVEIS GLOBAIS =====================
void parar(const char* motivo);

volatile long posicao = 0;
long limiteB = 0;
float passosPorM = PASSOS_POR_M_PADRAO;
volatile long limiteInfA = -1000000000L;
volatile long limiteSupB =  1000000000L;

// ---------- Encoder ----------
volatile long contagemEnc = 0;
volatile uint8_t estadoEnc = 0;
const int8_t tabelaEnc[16] = { 0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0 };

void IRAM_ATTR lerEncoder() {
  uint8_t e = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  contagemEnc += tabelaEnc[(estadoEnc << 2) | e];
  estadoEnc = e;
}

long lerContagemEnc() { noInterrupts(); long c = contagemEnc; interrupts(); return c; }
void zerarEncoder()   { noInterrupts(); contagemEnc = 0; interrupts(); }

// ---------- Gerador de passos por timer (igual ao pid_IA) ----------
const uint32_t TICK_HZ = 40000;
volatile uint32_t incFase = 0;
volatile int8_t dirCmd = 0;
hw_timer_t* timerPassos = nullptr;

void IRAM_ATTR onTimer() {
  static uint32_t fase = 0;
  static bool pulsoAlto = false;
  static int dirAtual = -1;
  bool acabouPulso = pulsoAlto;
  if (pulsoAlto) { digitalWrite(PIN_STEP, LOW); pulsoAlto = false; }
  int8_t d = dirCmd;
  if (d == 0 || digitalRead(PIN_FC_A) == ACIONADO || digitalRead(PIN_FC_B) == ACIONADO) { fase = 0; return; }
  int sentido = (d > 0) ? DIR_PARA_B : DIR_PARA_A;
  if (sentido != dirAtual) { digitalWrite(PIN_DIR, sentido); dirAtual = sentido; fase = 0; return; }
  if ((d > 0 && posicao >= limiteSupB) || (d < 0 && posicao <= limiteInfA)) { fase = 0; return; }
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
  if (a < 1.0f) { dirCmd = 0; incFase = 0; return; }
  incFase = (uint32_t)(a * 65536.0f / TICK_HZ);
  dirCmd = (vp > 0) ? 1 : -1;
}

// ---------- Homing (bloqueante, antes do timer) ----------
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

void andarPassos(long n, int sentido, long intervaloUs) { for (long i = 0; i < n; i++) darPasso(sentido, intervaloUs); }

void parar(const char* motivo) {
  dirCmd = 0; incFase = 0;
  digitalWrite(PIN_EN, HIGH);
  Serial.print("ERRO: "); Serial.println(motivo);
  while (true) delay(1000);
}

// ---------- Ângulo e velocidade angular (igual ao pid_IA) ----------
const int N_HIST = 8;
float phiHist[N_HIST];
uint32_t tHist[N_HIST];
int idxHist = 0;
float phi = 0;     // rad, 0 = pendurado, cresce para o lado B
float omega = 0;   // rad/s

float anguloRad() { return ENC_SINAL * (float)lerContagemEnc() * (2.0f * PI / CONTAGENS_POR_VOLTA); }

void iniciarHistorico() {
  phi = anguloRad(); idxHist = 0;
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
  phiHist[idxHist] = phi; tHist[idxHist] = t;
  idxHist = (idxHist + 1) % N_HIST;
}

float anguloGraus() { float d = fmodf(phi * 180.0f / PI, 360.0f); return d < 0 ? d + 360.0f : d; }
static inline float limitar(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------- Troca de dados entre os núcleos ----------
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool pnmpcAtivo = false;       // núcleo 1 liga/desliga; núcleo 0 obedece
volatile bool pnmpcZerar = false;
float shS[4] = {0, 0, 0, 0};            // medição mais recente [x v th w]   (protegido por mux)
float shA = 0;                          // a_cmd valendo agora [m/s^2]       (protegido por mux)
float shANext = 0;                      // a_cmd calculado para o PRÓXIMO período
bool  shTemNext = false;
uint32_t shAtivaUs = 0;                 // instante (micros) em que shANext passa a valer
uint32_t shAms = 0, shTus = 0;          // quando o a_cmd atual entrou / quanto levou o cálculo
int shModo = 0; float shE = -1;
uint32_t pnmpcAtrasos = 0;              // vezes em que o resultado chegou depois do início do período

// Comando t: mede o tempo do PNMPC no próprio ESP32, SEM mover o motor (só com o PNMPC desligado).
volatile bool pedirBench = false;
volatile uint32_t ciclosPNMPC = 0;      // diagnóstico: ciclos da tarefa do núcleo 0
volatile int etapaBench = 0;            // diagnóstico: onde o teste de tempo está
void medirTempoPNMPC() {
  const float estados[4][4] = { {0.0f, 0.0f, 0.3f, 2.0f}, {0.05f, 0.2f, 2.0f, -4.0f},
                                {0.0f, 0.0f, PI - 0.1f, 0.5f}, {-0.1f, -0.3f, PI + 0.2f, -1.0f} };
  uint32_t soma = 0, maximo = 0; int n = 0;
  mpc_reiniciar();
  etapaBench = 1;
  for (int r = 0; r < 15; r++)
    for (int e = 0; e < 4; e++) {
      etapaBench = 2 + r;
      float sp[4]; kan_prever(estados[e], 0.0f, sp);
      MpcInfo info;
      uint32_t t0 = micros();
      pnmpc_passo(sp, X_REF, TH_REF, &info);
      uint32_t dt = micros() - t0 + 0;
      soma += dt; if (dt > maximo) maximo = dt; n++;
    }
  mpc_reiniciar();
  Serial.printf("Tempo do PNMPC-KAN neste ESP32 (nucleo %d): media %lu us, max %lu us, periodo %lu us -> %s\n",
                xPortGetCoreID(), (unsigned long)(soma / n), (unsigned long)maximo,
                (unsigned long)(KAN_TS * 1e6f), maximo < 0.9f * KAN_TS * 1e6f ? "OK" : "LENTO DEMAIS");
}

// Compensação do atraso de cálculo: no início do período k o PNMPC lê a medição s(k), prevê com a
// KAN o estado em k+1 usando o a_cmd que vale durante [k, k+1) e otimiza a partir dele. O resultado
// passa a valer exatamente no início de k+1 (o laço de 1 kHz faz a troca), e não "quando ficar pronto".
void tarefaPNMPC(void*) {
  const TickType_t periodo = pdMS_TO_TICKS((uint32_t)(KAN_TS * 1000.0f + 0.5f));
  const uint32_t periodoUs = (uint32_t)(KAN_TS * 1e6f + 0.5f);
  Serial.printf("Tarefa PNMPC iniciada no nucleo %d\n", xPortGetCoreID());
  TickType_t ultimo = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&ultimo, periodo);
    ciclosPNMPC = ciclosPNMPC + 1;
    const uint32_t tk = micros();
    if (pnmpcZerar) { mpc_reiniciar(); pnmpcZerar = false; obsOk = false; aAnterior = 0.0f; }
    if (pedirBench && !pnmpcAtivo) { medirTempoPNMPC(); pedirBench = false; ultimo = xTaskGetTickCount(); continue; }
    if (!pnmpcAtivo) continue;
    float s[4], aAgora;
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < 4; i++) s[i] = shS[i];
    aAgora = shTemNext ? shANext : shA;          // o que vale durante [k, k+1)
    portEXIT_CRITICAL(&mux);
    // Observador (modelo em paralelo corrigido pela medição, como na figura do PNMPCI):
    // a KAN prevê theta e omega com o a_cmd aplicado e só o ÂNGULO medido corrige a previsão.
    // O omega do encoder na bancada é contaminado pelos trancos do motor (oscila ±2 rad/s a cada
    // amostra) e o PNMPC entrava em chattering de 50 Hz perseguindo esse ruído.
    if (!obsOk) {
      for (int i = 0; i < 4; i++) so[i] = s[i];
      obsOk = true;
    } else {
      float sp[4];
      kan_prever(so, aAnterior, sp);
      const float e = kan_wrap_pi(s[2] - sp[2]);
      so[0] = s[0]; so[1] = s[1];                // x e v vêm dos passos do motor (exatos)
      so[2] = sp[2] + OBS_L_TH * e;
      so[3] = sp[3] + OBS_L_W * e;
      if (!isfinite(so[2]) || !isfinite(so[3])) { for (int i = 0; i < 4; i++) so[i] = s[i]; }
    }
    aAnterior = aAgora;
    float sPrev[4];
    kan_prever(so, aAgora, sPrev);               // estado previsto em k+1 (compensa o cálculo)
    MpcInfo info;
    float a = pnmpc_passo(sPrev, X_REF, TH_REF, &info);
    const uint32_t dt = micros() - tk;
    if (!isfinite(a)) a = 0;
    portENTER_CRITICAL(&mux);
    shANext = a; shTemNext = true; shAtivaUs = tk + periodoUs;
    shTus = dt; shModo = info.modo; shE = info.E;
    portEXIT_CRITICAL(&mux);
    if (dt > periodoUs) pnmpcAtrasos++;
  }
}

// ---------- Controle (núcleo 1, 1 kHz) ----------
enum Modo { PARADO, IR_PARA, ESPERAR_REPOUSO, AGUARDAR, PNMPC, TESTE_SINAL };
int tentativas = 0;
uint32_t repousoDesde = 0;
Modo modo = PARADO;
Modo aposCentro = PARADO;
uint32_t chegouCentroEm = 0;
float vMs = 0;
long alvoPassos = 0;
bool zeroDefinido = false;
bool logLigado = false;
uint32_t tCtl = 0;

void rampaPara(float vDes, float aMax, float dt) {
  float dv = vDes - vMs, passo = aMax * dt;
  if (dv > passo) dv = passo; else if (dv < -passo) dv = -passo;
  vMs += dv;
}

float xMetros() { return (posicao - limiteB / 2) / passosPorM; }

void mostrarPosicao() {
  Serial.printf("Posicao: %ld passos (%.0f mm do centro) | Angulo: %.1f graus (sensor MATLAB: %.1f)\n",
                (long)posicao, xMetros() * 1000.0f, anguloGraus(), fmodf(anguloGraus() + 270.0f, 360.0f));
  Serial.printf("  tarefa PNMPC: %lu ciclos, teste de tempo na etapa %d\n", (unsigned long)ciclosPNMPC, etapaBench);
}

// Teste de sinal (comando k): com o pêndulo pendurado e parado, a = +A_TESTE (para B) por 150 ms e
// -A_TESTE por 150 ms (~1 cm). O modelo da KAN (th'' = -w0q sin th - kl a cos th) prevê que o ângulo
// DIMINUA logo após o empurrão para B. Se aumentar, a convenção da bancada difere da do modelo.
const float A_TESTE = 1.0f;
uint32_t tTeste0 = 0;
float thTeste0 = 0, somaTeste = 0; int nTeste = 0;

bool recentrar = false;          // depois de uma parada de segurança no PNMPC, volta ao centro sozinho
uint32_t paradoDesde = 0;

void pararControle(const char* msg, bool voltarAoCentro = false) {
  const bool estavaNoPNMPC = (modo == PNMPC);
  if (msg) Serial.println(msg);
  if (estavaNoPNMPC) {                       // diagnóstico: estado no instante da parada
    float a, E; uint32_t tus; int md;
    portENTER_CRITICAL(&mux); a = shA; E = shE; tus = shTus; md = shModo; portEXIT_CRITICAL(&mux);
    Serial.printf("  estado ao parar: x=%.0f mm v=%.3f m/s ang=%.1f graus w=%.2f rad/s a=%.2f %s E=%.2f calc=%lu us atrasos=%lu\n",
                  xMetros() * 1000.0f, vMs, anguloGraus(), omega, a, md ? "TOPO" : "SWING", E,
                  (unsigned long)tus, (unsigned long)pnmpcAtrasos);
  }
  pnmpcAtivo = false;
  modo = PARADO; aposCentro = PARADO;
  paradoDesde = millis();
  if (estavaNoPNMPC && voltarAoCentro) recentrar = true;
}

void controle(float dt) {
  atualizarAngulo();
  const float x = xMetros();
  const float vMax = V_MAX_PASSOS / passosPorM;

  if (fimDeCurso(PIN_FC_A) || fimDeCurso(PIN_FC_B)) {
    if (modo != PARADO) pararControle("Fim de curso acionado! Parando.", true);
    vMs = 0; aplicarVelocidade(0);
    if (recentrar && millis() - paradoDesde > 3000) {
      Serial.println("Fim de curso continua acionado: afaste o carrinho com a mao (driver desligado) e envie c.");
      recentrar = false;
    }
    return;
  }

  float a = 0;
  switch (modo) {
    case PARADO:
      rampaPara(0, A_FREIO_MS2, dt);
      // recuperação: parado há 1 s depois de uma parada de segurança -> volta ao centro
      if (recentrar && fabsf(vMs) < 0.001f && millis() - paradoDesde > 1000) {
        recentrar = false;
        alvoPassos = limiteB / 2; modo = IR_PARA;
        if (INICIO_AUTOMATICO && tentativas < MAX_TENTATIVAS) {
          aposCentro = ESPERAR_REPOUSO;
          Serial.printf("Recentralizando; nova tentativa automatica (%d de %d) quando o pendulo parar.\n",
                        tentativas + 1, MAX_TENTATIVAS);
        } else {
          aposCentro = PARADO;
          Serial.println("Recentralizando. Tentativas automaticas esgotadas: envie i para tentar de novo.");
        }
      }
      break;

    case IR_PARA: {
      float dPassos = (float)(alvoPassos - posicao), d = dPassos / passosPorM, vDes = 0;
      if (fabsf(dPassos) > 2) {
        vDes = sqrtf(2.0f * 0.8f * A_IR_MS2 * fabsf(d));
        if (vDes > V_IR_MS) vDes = V_IR_MS;
        if (d < 0) vDes = -vDes;
      }
      rampaPara(vDes, A_IR_MS2, dt);
      if (fabsf(dPassos) <= 2 && fabsf(vMs) < 0.005f) {
        modo = aposCentro; aposCentro = PARADO;
        chegouCentroEm = millis(); repousoDesde = millis(); vMs = 0;
        Serial.println("Chegou."); mostrarPosicao();
        if (modo == ESPERAR_REPOUSO) Serial.println("Esperando o pendulo ficar parado para ligar o PNMPC-KAN...");
      }
      break;
    }

    case ESPERAR_REPOUSO:                 // modo autônomo: espera o pêndulo pendurado e parado
      vMs = 0;
      if (fabsf(omega) > OMEGA_REPOUSO) repousoDesde = millis();
      if (millis() - repousoDesde >= T_REPOUSO_MS) {
        if (!zeroDefinido) {
          zerarEncoder(); iniciarHistorico(); zeroDefinido = true;
          Serial.println("Zero automatico: pendulo pendurado = 0 graus.");
        }
        tentativas++;
        chegouCentroEm = millis(); modo = AGUARDAR;
      }
      break;

    case AGUARDAR:                        // 2 s parado no centro antes de ligar o PNMPC
      vMs = 0;
      if (millis() - chegouCentroEm >= 2000) {
        pnmpcZerar = true;
        portENTER_CRITICAL(&mux);
        shS[0] = x; shS[1] = 0; shS[2] = phi; shS[3] = omega;
        shA = 0; shANext = 0; shTemNext = false; shAms = millis();
        portEXIT_CRITICAL(&mux);
        pnmpcAtivo = true;
        modo = PNMPC;
        Serial.println("PNMPC-KAN ligado: swing-up + equilibrio em pe.");
      }
      break;

    case TESTE_SINAL: {
      const uint32_t e = millis() - tTeste0;
      a = (e < 150) ? A_TESTE : (e < 300 ? -A_TESTE : 0.0f);
      if (e < 250) { somaTeste += kan_wrap_pi(phi - thTeste0); nTeste++; }
      if (e >= 320) {
        const float media = nTeste ? somaTeste / nTeste * 180.0f / PI : 0.0f;
        Serial.printf("Teste de sinal: empurrao para B (a > 0) -> angulo variou em media %+.3f graus.\n", media);
        if (fabsf(media) < 0.05f)
          Serial.println("  Resposta pequena demais para concluir (pendulo preso? encoder?). Repita com k.");
        else if (media < 0)
          Serial.println("  OK: o angulo DIMINUIU, como o modelo da KAN preve. Convencao certa.");
        else
          Serial.println("  INVERTIDO: o angulo AUMENTOU, o modelo preve o contrario. Troque ENC_SINAL (-1 <-> 1) e grave de novo.");
        vMs = 0; a = 0;
        modo = PARADO; paradoDesde = millis(); recentrar = true;
      }
      break;
    }

    case PNMPC: {
      uint32_t idade;
      portENTER_CRITICAL(&mux);
      shS[0] = x; shS[1] = vMs; shS[2] = phi; shS[3] = omega;   // medição p/ o próximo cálculo
      if (shTemNext && (int32_t)(micros() - shAtivaUs) >= 0) {  // novo a_cmd entra no início do período
        shA = shANext; shTemNext = false; shAms = millis();
      }
      a = shA; idade = millis() - shAms;
      portEXIT_CRITICAL(&mux);
      if (idade > PNMPC_TIMEOUT_MS) { pararControle("PNMPC sem resposta (calculo atrasado). Parado.", true); break; }
      // limite de software do curso (o timer para de dar passos ali): não deixa o PNMPC "empurrar a parede"
      if ((posicao >= limiteSupB - 40 && vMs > 0.0f) || (posicao <= limiteInfA + 40 && vMs < 0.0f)) {
        pararControle("Carrinho chegou ao limite do curso durante o PNMPC. Parado.", true);
        break;
      }
      a = limitar(a, MPC.u_min, MPC.u_max);
      break;
    }
  }

  if (modo == PNMPC || modo == TESTE_SINAL) {
    vMs = limitar(vMs + a * dt, -vMax, vMax);
    // Freio de parede: nunca mais rápido do que consegue frear antes da extremidade
    float dB = (limiteB - posicao) / passosPorM - MARGEM_M;
    float dA = posicao / passosPorM - MARGEM_M;
    if (dB < 0) dB = 0;
    if (dA < 0) dA = 0;
    float vB = sqrtf(2.0f * A_FREIO_MS2 * dB), vA = sqrtf(2.0f * A_FREIO_MS2 * dA);
    if (vMs > vB) vMs = vB;
    if (vMs < -vA) vMs = -vA;
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
    if (modo == PNMPC || modo == TESTE_SINAL) {   // só é perigoso com o controlador atuando
      pararControle("Controle atrasou mais de 10 ms. Parado.", true);
      vMs = 0; aplicarVelocidade(0);
      return;
    }
    dt = 0.010f;                                  // nos outros modos só limita o passo de integração
  }
  controle(dt);
}

void tratarComando(char c) {
  switch (tolower(c)) {
    case 'i':
      if (!zeroDefinido) { Serial.println("Pendulo para baixo e parado: envie z primeiro."); break; }
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("Pare com s e espere antes de iniciar."); break; }
      iniciarHistorico();
      alvoPassos = limiteB / 2; aposCentro = AGUARDAR; modo = IR_PARA;
      Serial.println("Centralizando; em 2 s liga o PNMPC-KAN.");
      break;
    case 's': pararControle("Parando."); break;
    case 'a': case 'b': case 'c':
      pnmpcAtivo = false; aposCentro = PARADO;
      alvoPassos = tolower(c) == 'a' ? 0 : (tolower(c) == 'b' ? limiteB : limiteB / 2);
      modo = IR_PARA;
      break;
    case 'z':
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("Pare antes de zerar (comando s)."); break; }
      zerarEncoder(); iniciarHistorico(); zeroDefinido = true;
      Serial.println("Zero definido: pendulo pendurado = 0 graus.");
      break;
    case 'p': mostrarPosicao(); break;
    case 'k':
      if (!zeroDefinido) { Serial.println("Pendulo para baixo e parado: envie z primeiro."); break; }
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("Pare (s) e espere o carrinho parar."); break; }
      if (fabsf(kan_wrap_pi(phi)) > 0.05f || fabsf(omega) > 0.2f) { Serial.println("O pendulo precisa estar pendurado e parado."); break; }
      if (fabsf(xMetros()) > 0.10f) { Serial.println("Leve o carrinho ao centro (c) antes do teste."); break; }
      iniciarHistorico();
      thTeste0 = phi; somaTeste = 0; nTeste = 0; tTeste0 = millis();
      modo = TESTE_SINAL;
      Serial.println("Teste de sinal: empurrao curto para B (~1 cm)...");
      break;
    case 't':
      if (modo == PNMPC) { Serial.println("Pare (s) antes de medir o tempo."); break; }
      Serial.println("Medindo o tempo do PNMPC-KAN (o motor nao se move)...");
      pedirBench = true;
      break;
    case 'l':
      logLigado = !logLigado;
      if (logLigado) Serial.println("t_ms,x_mm,v_ms,ang_graus,omega_rads,a_cmd,modo,E,t_us");
      break;
    case '\n': case '\r': case ' ': break;
    default:
      Serial.println("Comandos: z (zero), i (PNMPC-KAN), s (parar), a/b/c (mover), p (posicao), l (log), t (tempo do PNMPC), k (teste de sinal)");
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.setTxBufferSize(1024);   // mensagens não bloqueiam o laço de 1 kHz (sem isso uma linha trava ~7 ms)
  Serial.begin(115200);
  pinMode(PIN_STEP, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_FC_A, INPUT_PULLUP); pinMode(PIN_FC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP); pinMode(PIN_ENC_B, INPUT_PULLUP);

  estadoEnc = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), lerEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), lerEncoder, CHANGE);

  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_EN, LOW);
  delay(500);

  Serial.printf("PNMPC-KAN bancada | KAN %s %d-%d-%d | N = %d, %d movimentos, %d iteracao(oes)\n",
                KAN_NOME, KAN_TAM[0], KAN_TAM[1], KAN_TAM[2], MPC_N, MPC_NU, MPC.iters);

  if (fimDeCurso(PIN_FC_A) && fimDeCurso(PIN_FC_B))
    parar("os dois fins de curso estao acionados (fio solto ou logica errada)");

  Serial.println("Procurando fim de curso A...");
  if (!fimDeCurso(PIN_FC_A)) {
    if (andarAteFimDeCurso(PIN_FC_A, DIR_PARA_A, INTERVALO_HOMING_US) < 0) parar("fim de curso A nao encontrado");
  }
  andarPassos(RECUO_PASSOS, DIR_PARA_B, INTERVALO_HOMING_US);
  if (fimDeCurso(PIN_FC_A)) parar("FC A nao liberou apos recuo");
  posicao = 0;

  Serial.println("Medindo ate o fim de curso B...");
  if (andarAteFimDeCurso(PIN_FC_B, DIR_PARA_B, INTERVALO_MEDIDA_US) < 0) parar("fim de curso B nao encontrado");
  long distanciaPassos = posicao;
  andarPassos(RECUO_PASSOS, DIR_PARA_A, INTERVALO_HOMING_US);
  if (fimDeCurso(PIN_FC_B)) parar("FC B nao liberou apos recuo");
  limiteB = posicao;
  if (limiteB <= 2 * RECUO_PASSOS) parar("curso medido invalido");
  if (CURSO_REAL_M > 0) passosPorM = (distanciaPassos + RECUO_PASSOS) / CURSO_REAL_M;

  const float meioCurso = 0.5f * limiteB / passosPorM;
  MPC.x_soft = meioCurso - FOLGA_SOFT_M;          // penalidade de trilho do PNMPC pelo curso real
  if (MPC.x_soft < 0.03f) MPC.x_soft = 0.03f;
  // NÃO modelar a saturação de velocidade no PNMPC: com v saturada o gradiente por perturbação zera e
  // ele deixa de frear (na simulação isso levou o carrinho ao fim de curso em < 1 s).
  MPC.v_max = 0.0f;
  MPC.v_soft = 0.8f * V_MAX_PASSOS / passosPorM;
  MPC.q_vs = 0.0f;   // penalidade de velocidade desligada: com o acoplamento real (kl = 2,1) ela atrapalhava o equilíbrio
  Serial.printf("Velocidade maxima do carrinho: %.2f m/s\n", V_MAX_PASSOS / passosPorM);
  Serial.printf("Curso: %ld passos | %.0f passos/m (%s) | curso util %.0f mm | x_soft do PNMPC %.0f mm\n",
                distanciaPassos, passosPorM, CURSO_REAL_M > 0 ? "medido" : "padrao - ajuste CURSO_REAL_M",
                2000.0f * meioCurso, 1000.0f * MPC.x_soft);
  if (meioCurso < 0.18f)
    Serial.println("AVISO: curso util menor que 360 mm; o swing-up foi ajustado com +-200 mm e pode bater no fim.");

  limiteInfA = 0; limiteSupB = limiteB;
  iniciarHistorico();
  iniciarTimer();
  tCtl = micros();

  // PNMPC no núcleo 0 (o loop e as ISRs de passo/encoder ficam no núcleo 1)
  if (xTaskCreatePinnedToCore(tarefaPNMPC, "PNMPC", 12288, NULL, 5, NULL, 0) != pdPASS) parar("nao consegui criar a tarefa do PNMPC");

  alvoPassos = limiteB / 2; modo = IR_PARA; zeroDefinido = false;
  if (INICIO_AUTOMATICO) { aposCentro = ESPERAR_REPOUSO; repousoDesde = millis(); }
  Serial.println("Indo ao meio. Depois de chegar e o pendulo parar para baixo, envie z; depois i.");
  tCtl = micros();   // zera o relógio do laço DEPOIS das mensagens: senão o 1o ciclo vê > 10 ms e para
}

// ===================== LOOP (núcleo 1) =====================
void loop() {
  if (Serial.available()) tratarComando(Serial.read());
  tarefaControle();

  static uint32_t tPrint = 0, tLog = 0;
  if (modo == PNMPC) {
    float a, E; uint32_t tus; int md;
    portENTER_CRITICAL(&mux); a = shA; E = shE; tus = shTus; md = shModo; portEXIT_CRITICAL(&mux);
    char linha[140]; int n = 0;
    if (logLigado && millis() - tLog >= 10) {
      tLog = millis();
      n = snprintf(linha, sizeof(linha), "%lu,%.1f,%.4f,%.2f,%.3f,%.3f,%d,%.3f,%lu\n", (unsigned long)tLog,
                   xMetros() * 1000.0f, vMs, anguloGraus(), omega, a, md, E, (unsigned long)tus);
    } else if (!logLigado && millis() - tPrint > 200) {
      tPrint = millis();
      n = snprintf(linha, sizeof(linha), "%s ang=%.1f x=%.0fmm v=%.3f a=%.2f E=%.2f calc=%luus atrasos=%lu\n",
                   md ? "TOPO " : "SWING", anguloGraus(), xMetros() * 1000.0f, vMs, a, E,
                   (unsigned long)tus, (unsigned long)pnmpcAtrasos);
    }
    if (n > 0 && n < (int)sizeof(linha) && Serial.availableForWrite() >= n) Serial.write((const uint8_t*)linha, n);
  }
}
