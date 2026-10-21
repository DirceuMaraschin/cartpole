// ESP32 - Pêndulo invertido sobre carrinho com motor de passo (STEP/DIR, microstep 1/8)
// Homing + centro + swing-up + PID de equilibrio em 180 graus
// >>> VERSAO COM MODO DE LOG para identificacao do modelo (etapa 1 do projeto PPO) <<<
//
// O controle NAO foi alterado: o PID continua exatamente como estava.
// Foi acrescentado um registrador de dados que amostra o estado a 100 Hz
// e envia por serial em formato CSV, sem bloquear o laco de 1 kHz.
//
// Convenção de ângulo: 0°/360° = pêndulo pendurado (pra baixo), 180° = em pé.
// Convenção de lado: o sentido A -> B do carrinho é o "positivo".
//
// Ligações:
//   STEP -> 25 | DIR -> 26 | EN -> 27
//   Fim de curso A -> 32 e GND | Fim de curso B -> 33 e GND (NF, pull-up interno)
//   Encoder A -> 18 | Encoder B -> 19 (pull-up externo 4.7k para 3.3V)
//
// Comandos (Monitor Serial 921600):
//   i = centralizar e iniciar swing-up + PID    s = parar
//   e = centralizar e aguardar elevacao MANUAL para testar apenas o PID
//   a / b / c = ir ao lado A / lado B / centro
//   z = definir 0° (pêndulo pendurado e parado)    p = mostrar posição e ângulo
//   l = liga/desliga o LOG em CSV (funciona em qualquer modo)
//   k = imprime as constantes do sistema (cabecalho do CSV)

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
const int ENC_SINAL = -1;   // inverta para +1 se o ângulo cresce para o lado errado

// ===================== MECÂNICA (AJUSTE!) =====================
// MEÇA COM RÉGUA a distância entre os pontos onde os dois fins de curso acionam
// e preencha abaixo, em metros. Com 0 usa o valor teorico (40000 passos/m).
// Sem essa medida, o "x" que o agente recebe tem erro de escala.
const float CURSO_REAL_M = 0.0;
const float PASSOS_POR_M_PADRAO = 40000.0;
// Comprimento equivalente do pêndulo, em metros. Haste uniforme: 2/3 do comprimento.
const float L_EQ_M = 0.220;

// ===================== LIMITES DE MOVIMENTO =====================
const float V_MAX_PASSOS = 18000.0;  // passos/s
const float A_IR_MS2     = 1.0;
const float V_IR_MS      = 0.15;
const float A_FREIO_MS2  = 8.0;
const float MARGEM_M     = 0.01;

// ===================== SWING-UP =====================
const float K_SW     = 6.0;
const float A_SW_MAX = 4.0;
const float KX_SW    = 1.0;
const float KV_SW    = 0.1;

// ===================== EQUILÍBRIO =====================
const float CAPTURA_GRAUS = 20.0;
const float CAPTURA_VEL   = 5.0;
const float SAIDA_GRAUS   = 35.0;
const float KP = 40;
const float KI = 1;
const float I_MAX = 0.5;
const float KD = 5.5;
const float KX = 1.5;
const float KV = 17.5;
const float A_BAL_MAX = 8.0;

// ===================== VARIÁVEIS GLOBAIS =====================
void parar(const char* motivo);

volatile long posicao = 0;
long limiteB = 0;
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

// ---------- Gerador de passos por timer ----------
const uint32_t TICK_HZ = 40000;
volatile uint32_t incFase = 0;
volatile int8_t dirCmd = 0;
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
  if (d == 0 || digitalRead(PIN_FC_A) == ACIONADO ||
      digitalRead(PIN_FC_B) == ACIONADO) { fase = 0; return; }
  int sentido = (d > 0) ? DIR_PARA_B : DIR_PARA_A;
  if (sentido != dirAtual) {
    digitalWrite(PIN_DIR, sentido);
    dirAtual = sentido;
    fase = 0;
    return;
  }
  if ((d > 0 && posicao >= limiteSupB) ||
      (d < 0 && posicao <= limiteInfA)) { fase = 0; return; }

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

// ---------- Homing e medição ----------
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
float termoI = 0;
uint32_t chegouCentroEm = 0;

float vMs = 0;
long alvoPassos = 0;
bool zeroDefinido = false;
uint32_t tCtl = 0;

// ===================== LOG (NOVO) =====================
// Amostra o estado a 100 Hz dentro do laco de 1 kHz, guarda em um buffer
// circular e deixa o loop() esvaziar para a serial quando houver espaco.
// Assim nenhuma escrita serial atrasa o controle.
const int LOG_DIVISOR = 10;     // 1000 Hz / 10 = 100 Hz
const int LOG_BUF     = 128;

struct Amostra {
  uint32_t t;
  float x, v, phi, omega, a;
  uint8_t modo;
};

Amostra logBuf[LOG_BUF];
volatile int logCab = 0;
volatile int logCau = 0;
bool     logAtivo    = false;
int      logCont     = 0;
uint32_t logT0       = 0;
long     logPerdidas = 0;

void registrarAmostra(float x, float v, float a) {
  int prox = (logCab + 1) % LOG_BUF;
  if (prox == logCau) { logPerdidas++; return; }  // buffer cheio: descarta
  Amostra &s = logBuf[logCab];
  s.t = micros();
  s.x = x;
  s.v = v;
  s.phi = phi;
  s.omega = omega;
  s.a = a;
  s.modo = (uint8_t)modo;
  logCab = prox;
}

void enviarLog() {
  char linha[128];
  while (logCau != logCab) {
    Amostra &s = logBuf[logCau];
    int n = snprintf(linha, sizeof(linha), "%.4f,%.5f,%.4f,%.5f,%.4f,%.4f,%u\n",
                     (double)((uint32_t)(s.t - logT0) * 1e-6f),
                     s.x, s.v, s.phi, s.omega, s.a, s.modo);
    if (n <= 0 || n >= (int)sizeof(linha)) { logCau = (logCau + 1) % LOG_BUF; continue; }
    if (Serial.availableForWrite() < n) return;   // sem espaco: tenta na proxima volta
    Serial.write((const uint8_t*)linha, n);
    logCau = (logCau + 1) % LOG_BUF;
  }
}

void imprimirConstantes() {
  Serial.println("# pendulo_invertido_log v1");
  Serial.printf("# passos_por_m=%.1f\n", passosPorM);
  Serial.printf("# curso_passos=%ld\n", limiteB);
  Serial.printf("# curso_m=%.4f\n", limiteB / passosPorM);
  Serial.printf("# curso_real_medido=%s\n", CURSO_REAL_M > 0 ? "sim" : "nao");
  Serial.printf("# l_eq_m=%.4f\n", L_EQ_M);
  Serial.printf("# w0_rads=%.4f\n", W0);
  Serial.printf("# contagens_por_volta=%d\n", CONTAGENS_POR_VOLTA);
  Serial.printf("# passos_por_volta=%d\n", PASSOS_POR_VOLTA);
  Serial.printf("# v_max_ms=%.4f\n", V_MAX_PASSOS / passosPorM);
  Serial.printf("# a_bal_max=%.2f\n", A_BAL_MAX);
  Serial.printf("# a_freio=%.2f\n", A_FREIO_MS2);
  Serial.printf("# kp=%.3f ki=%.3f kd=%.3f kx=%.3f kv=%.3f\n", KP, KI, KD, KX, KV);
  Serial.printf("# f_controle_hz=1000 f_log_hz=%d\n", 1000 / LOG_DIVISOR);
  Serial.println("# filtro_omega=janela8amostras+ema0.5");
  Serial.println("# modo: 0=PARADO 1=IR_PARA 2=AGUARDAR_TOPO 3=SWINGUP 4=EQUILIBRIO");
  Serial.println("# psi = wrap(pi - phi)   psiDot = -omega");
}

void alternarLog() {
  logAtivo = !logAtivo;
  if (logAtivo) {
    logCab = logCau = 0;
    logCont = 0;
    logPerdidas = 0;
    logT0 = micros();
    imprimirConstantes();
    Serial.println("t_s,x_m,v_ms,phi_rad,omega_rads,a_ms2,modo");
  } else {
    Serial.printf("# fim do log, amostras perdidas=%ld\n", logPerdidas);
  }
}
// ===================== FIM DO BLOCO DE LOG =====================

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
    if (modo != PARADO) Serial.println("# Fim de curso acionado! Parando.");
    modo = PARADO;
    aposCentro = PARADO;
    termoI = 0;
    vMs = 0;
    aplicarVelocidade(0);
    if (logAtivo && ++logCont >= LOG_DIVISOR) { logCont = 0; registrarAmostra(x, vMs, 0); }
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
        Serial.println("# Chegou.");
      }
      break;
    }

    case AGUARDAR_TOPO: {
      vMs = 0;
      float psi = wrapPi(PI - phi);
      if (usarSwingup) {
        if (millis() - chegouCentroEm >= 2000) modo = SWINGUP;
      } else if (fabsf(psi) < CAPTURA_GRAUS * PI / 180.0f &&
                 fabsf(omega) < CAPTURA_VEL) {
        termoI = 0;
        modo = EQUILIBRIO;
        Serial.println("# PID capturou o pendulo.");
      }
      break;
    }

    case SWINGUP: {
      float psi = wrapPi(PI - phi);
      float psiDot = -omega;

      float En = 0.5f * (omega / W0) * (omega / W0) - cosf(phi);
      float dEn = 1.0f - En;
      float s = omega * cosf(phi);
      float sg = (fabsf(omega) < 0.2f) ? 1.0f : ((s > 0) ? 1.0f : -1.0f);
      float u = limitar(K_SW * dEn, -A_SW_MAX, A_SW_MAX);
      a = -u * sg - KX_SW * x - KV_SW * vMs;

      if (fabsf(psi) < CAPTURA_GRAUS * PI / 180.0f && fabsf(psiDot) < CAPTURA_VEL) {
        modo = EQUILIBRIO;
        termoI = 0;
        Serial.println("# Capturado! Equilibrando...");
      }
      break;
    }

    case EQUILIBRIO: {
      float psi = wrapPi(PI - phi);
      float psiDot = -omega;
      if (fabsf(psi) > SAIDA_GRAUS * PI / 180.0f) {
        termoI = 0;
        modo = usarSwingup ? SWINGUP : PARADO;
        if (!usarSwingup) { vMs = 0; aplicarVelocidade(0); }
        Serial.println("# Perdeu equilibrio.");
        break;
      }
      float base = KP * psi + KD * psiDot + KX * x + KV * vMs;
      float candidato = (KI > 0) ? limitar(termoI + KI * psi * dt, -I_MAX, I_MAX) : 0;
      float bruto = base + candidato;
      saturou = fabsf(bruto) > A_BAL_MAX;
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

    float dB = (limiteB - posicao) / passosPorM - MARGEM_M;
    float dA = posicao / passosPorM - MARGEM_M;
    if (dB < 0) dB = 0;
    if (dA < 0) dA = 0;
    float vB = sqrtf(2.0f * A_FREIO_MS2 * dB);
    float vA = sqrtf(2.0f * A_FREIO_MS2 * dA);
    if (vMs > vB) { vMs = vB; limitouVelocidade = true; }
    if (vMs < -vA) { vMs = -vA; limitouVelocidade = true; }
    if (calculouPID && limitouVelocidade) termoI = integralAnterior;
  }

  vMs = limitar(vMs, -vMax, vMax);
  aplicarVelocidade(vMs);

  // --- amostragem do log (100 Hz) ---
  if (logAtivo && ++logCont >= LOG_DIVISOR) {
    logCont = 0;
    registrarAmostra(x, vMs, a);
  }
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
    Serial.println("# Controle atrasou mais de 10 ms. Parado; reinicie com i/e.");
    return;
  }
  controle(dt);
}

void tratarComando(char c) {
  switch (tolower(c)) {
    case 'i':
    case 'e':
      if (!zeroDefinido) { Serial.println("# Pendulo para baixo e parado: envie z primeiro."); break; }
      if (modo != PARADO || fabsf(vMs) > 0.001f) {
        Serial.println("# Pare com s e espere antes de iniciar."); break;
      }
      iniciarHistorico();
      termoI = 0;
      usarSwingup = (tolower(c) == 'i');
      alvoPassos = limiteB / 2;
      aposCentro = AGUARDAR_TOPO;
      modo = IR_PARA;
      Serial.println(usarSwingup ? "# Centralizando para swing-up + PID." :
        "# Centralizando. Ao chegar, eleve o pendulo.");
      break;
    case 's':
      modo = PARADO;
      aposCentro = PARADO;
      termoI = 0;
      Serial.println("# Parando.");
      break;
    case 'a': case 'b': case 'c':
      termoI = 0;
      aposCentro = PARADO;
      alvoPassos = tolower(c) == 'a' ? 0 : (tolower(c) == 'b' ? limiteB : limiteB / 2);
      modo = IR_PARA;
      break;
    case 'z':
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("# Pare antes de zerar (comando s)."); break; }
      zerarEncoder();
      iniciarHistorico();
      zeroDefinido = true;
      Serial.println("# Zero definido: pendulo pendurado = 0 graus.");
      break;
    case 'p': mostrarPosicao(); break;
    case 'l': alternarLog(); break;
    case 'k': imprimirConstantes(); break;
    case '\n': case '\r': case ' ': break;
    default:
      Serial.println("# Comandos: i e s a b c z p | l = log CSV | k = constantes");
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(921600);

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

  Serial.println("# Procurando fim de curso A...");
  if (!fimDeCurso(PIN_FC_A)) {
    if (andarAteFimDeCurso(PIN_FC_A, DIR_PARA_A, INTERVALO_HOMING_US) < 0)
      parar("fim de curso A nao encontrado");
  }
  andarPassos(RECUO_PASSOS, DIR_PARA_B, INTERVALO_HOMING_US);
  if (fimDeCurso(PIN_FC_A)) parar("FC A nao liberou apos recuo");
  posicao = 0;

  Serial.println("# Medindo ate o fim de curso B...");
  if (andarAteFimDeCurso(PIN_FC_B, DIR_PARA_B, INTERVALO_MEDIDA_US) < 0)
    parar("fim de curso B nao encontrado");
  long distanciaPassos = posicao;
  andarPassos(RECUO_PASSOS, DIR_PARA_A, INTERVALO_HOMING_US);
  if (fimDeCurso(PIN_FC_B)) parar("FC B nao liberou apos recuo");
  limiteB = posicao;
  if (limiteB <= 2 * RECUO_PASSOS) parar("curso medido invalido");

  if (CURSO_REAL_M > 0) passosPorM = (distanciaPassos + RECUO_PASSOS) / CURSO_REAL_M;

  Serial.printf("# Curso: %ld passos | %.0f passos/m (%s) | curso util %.0f mm\n",
                distanciaPassos, passosPorM,
                CURSO_REAL_M > 0 ? "medido" : "padrao - ajuste CURSO_REAL_M",
                1000.0f * limiteB / passosPorM);

  limiteInfA = 0;
  limiteSupB = limiteB;

  iniciarHistorico();
  iniciarTimer();
  tCtl = micros();

  alvoPassos = limiteB / 2;
  modo = IR_PARA;
  zeroDefinido = false;
  Serial.println("# Indo ao meio. Depois de chegar e o pendulo parar para baixo, envie z.");
  Serial.println("# i = swing-up+PID | e = captura manual | s = parar | l = log CSV");
}

// ===================== LOOP =====================
void loop() {
  if (Serial.available()) tratarComando(Serial.read());
  tarefaControle();

  if (logAtivo) {
    enviarLog();
    return;              // durante o log, nada de texto para humano
  }

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
