// ============================================================================
// ESP32 - Pendulo invertido com motor de passo: PID + modo RL (agente PPO)
// ============================================================================
//
// O PID NAO foi alterado. Ele continua funcionando exatamente como antes e
// agora tambem serve de SUPERVISOR: se o agente perder a haste, o controle
// volta para o PID automaticamente, em vez de deixar cair.
//
// O que foi acrescentado:
//   - modo RL: a aceleracao vem do PC, por serial, a 50 Hz
//   - protocolo binario com CRC8, sem texto no laco de controle
//   - supervisor: |psi| > LIMITE_RL devolve o controle ao PID
//   - watchdog: se o PC parar de responder, o carrinho freia
//   - limitador de velocidade para os primeiros testes (comando v)
//
// Convencao de angulo: 0/360 graus = pendulo pendurado, 180 = em pe.
// Convencao de lado  : o sentido A -> B do carrinho e o "positivo".
//
// Ligacoes:
//   STEP -> 25 | DIR -> 26 | EN -> 27
//   Fim de curso A -> 32 e GND | Fim de curso B -> 33 e GND (NF, pull-up)
//   Encoder A -> 18 | Encoder B -> 19 (pull-up externo 4.7k para 3.3V)
//
// Comandos (Monitor Serial 921600):
//   i = swing-up + PID          e = centralizar e capturar manualmente
//   s = parar                   a/b/c = ir ao lado A / B / centro
//   z = definir zero            p = mostrar posicao e angulo
//   l = log CSV (so fora do modo RL)      k = constantes
//   r = entregar o controle ao AGENTE (modo RL)
//   m = liga/desliga o supervisor PID
//   v = limitador de velocidade: 50% -> 75% -> 100%
// ============================================================================

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

// ===================== CONFIGURACAO BASICA =====================
const int ACIONADO   = HIGH;
const int DIR_PARA_A = LOW;
const int DIR_PARA_B = HIGH;

const int  PASSOS_POR_VOLTA    = 200 * 8;
const long INTERVALO_HOMING_US = 800;
const long INTERVALO_MEDIDA_US = 500;
const long RECUO_PASSOS        = 80;
const long LIMITE_PASSOS       = 200000;

const int CONTAGENS_POR_VOLTA = 600 * 4;
const int ENC_SINAL = -1;

// ===================== MECANICA =====================
// MEDIR: distancia entre os pontos onde os dois fins de curso ACIONAM.
// Nao e a mesma coisa que "onde o carrinho encosta na chave".
// Veja o procedimento em identificacao.md secao 6.
const float CURSO_REAL_M = 0.0;
const float PASSOS_POR_M_PADRAO = 40000.0;
const float L_EQ_M = 0.2246;        // MEDIDO pela oscilacao livre (era 0.220)

// ===================== LIMITES DE MOVIMENTO =====================
const float V_MAX_PASSOS = 18000.0;
const float A_IR_MS2     = 1.0;
const float V_IR_MS      = 0.15;
const float A_FREIO_MS2  = 8.0;
const float MARGEM_M     = 0.01;

// ===================== SWING-UP =====================
const float K_SW     = 6.0;
const float A_SW_MAX = 4.0;
const float KX_SW    = 1.0;
const float KV_SW    = 0.1;

// ===================== EQUILIBRIO (PID) =====================
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

// ===================== MODO RL =====================
// Periodo de decisao do agente. DEVE bater com DT_CTRL do simulador.
const int   RL_DIVISOR     = 20;      // 1000 Hz / 20 = 50 Hz
// Supervisor: acima disso o PID retoma. Mais apertado que SAIDA_GRAUS de
// proposito -- o PID recebe a haste ainda recuperavel, com margem ate 35.
const float LIMITE_RL_GRAUS = 20.0;
// Watchdog: sem acao valida por este tempo, freia.
const uint32_t RL_TIMEOUT_MS = 60;    // 3 periodos perdidos
// Limite de posicao do agente (o firmware impoe, o PC nao decide).
const float X_LIM_RL = 0.24;

// Cabecalhos do protocolo
const uint16_t CAB_SUBIDA = 0xAA55;   // ESP32 -> PC
const uint16_t CAB_DESCIDA= 0xA55A;   // PC -> ESP32

// ===================== VARIAVEIS GLOBAIS =====================
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

// Fator do limitador de velocidade (comando v). Comece os testes em 0.5.
float fatorV = 0.5f;

void aplicarVelocidade(float v_ms) {
  float vp = v_ms * passosPorM;
  float teto = V_MAX_PASSOS * fatorV;
  if (vp >  teto) vp =  teto;
  if (vp < -teto) vp = -teto;
  float a = fabsf(vp);
  if (a < 1.0f) { dirCmd = 0; incFase = 0; return; }
  incFase = (uint32_t)(a * 65536.0f / TICK_HZ);
  dirCmd = (vp > 0) ? 1 : -1;
}

// ---------- Homing ----------
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

// ---------- Angulo e velocidade angular ----------
const int N_HIST = 8;
float phiHist[N_HIST];
uint32_t tHist[N_HIST];
int idxHist = 0;
float phi = 0;
float omega = 0;

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

// ===================== PROTOCOLO BINARIO =====================
//
// Por que binario e nao texto: o parsing de string custa tempo VARIAVEL nas
// duas pontas, e um pacote de texto fica 3 a 4x maior. Com 21 bytes a 50 Hz
// o trafego e de ~1,4 kB/s -- folga de varias ordens de grandeza.
//
// Subida (ESP32 -> PC), 21 bytes:
//   [0..1] 0xAA55   [2] seq   [3..6] posicao(int32)   [7..10] enc(int32)
//   [11..14] v(float)   [15..18] omega(float)   [19] flags   [20] crc8
//
// Descida (PC -> ESP32), 7 bytes:
//   [0..1] 0xA55A   [2] seq eco   [3..4] acao(int16, m/s2 x 4000)
//   [5] comando   [6] crc8

uint8_t crc8(const uint8_t* d, int n) {
  uint8_t c = 0x00;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// flags da subida
const uint8_t F_FC_A     = 0x01;
const uint8_t F_FC_B     = 0x02;
const uint8_t F_RL       = 0x04;   // esta no modo RL
const uint8_t F_SUPERV   = 0x08;   // supervisor PID assumiu neste ciclo
const uint8_t F_PERDEU   = 0x10;   // pacote anterior nao chegou a tempo
const uint8_t F_ZERO_OK  = 0x20;   // encoder ja foi zerado

// comandos da descida
const uint8_t CMD_NADA    = 0;
const uint8_t CMD_PARAR   = 1;
const uint8_t CMD_PID     = 2;     // devolve o controle ao PID
const uint8_t CMD_RL      = 3;     // (re)entra no modo RL

volatile float aRL = 0.0f;         // aceleracao vinda do PC, ja validada
float  aRL_pendente = 0.0f;        // recebida, esperando a fronteira de 20 ms
bool   aRL_nova = false;
uint32_t ultimaAcaoMs = 0;
uint8_t  seqAtual = 0;
uint16_t pacotesPerdidos = 0;
bool     supervisorLigado = true;
bool     modoBinario = false;      // true = nada de texto na serial

// --- maquina de estados do receptor, nao bloqueante ---
uint8_t rxBuf[7];
int     rxN = 0;

void processarSerialRL() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    // ressincronizacao pelo cabecalho
    if (rxN == 0) { if (b != 0x5A) continue; rxBuf[rxN++] = b; continue; }
    if (rxN == 1) { if (b != 0xA5) { rxN = 0; continue; } rxBuf[rxN++] = b; continue; }

    rxBuf[rxN++] = b;
    if (rxN < 7) continue;
    rxN = 0;

    if (crc8(rxBuf, 6) != rxBuf[6]) continue;      // corrompido: descarta

    int16_t bruto = (int16_t)((uint16_t)rxBuf[3] | ((uint16_t)rxBuf[4] << 8));
    float a = (float)bruto / 4000.0f;

    // o firmware NAO confia no PC: satura aqui tambem
    aRL_pendente = limitar(a, -A_BAL_MAX, A_BAL_MAX);
    aRL_nova = true;
    ultimaAcaoMs = millis();

    uint8_t cmd = rxBuf[5];
    extern void tratarComandoRL(uint8_t);
    if (cmd != CMD_NADA) tratarComandoRL(cmd);
  }
}

void enviarEstado(uint8_t flags) {
  uint8_t p[21];
  p[0] = 0x55; p[1] = 0xAA;
  p[2] = seqAtual++;
  long pos = posicao;
  long enc = lerContagemEnc();
  memcpy(&p[3],  &pos, 4);
  memcpy(&p[7],  &enc, 4);
  extern float vMs;
  float v = vMs, om = omega;
  memcpy(&p[11], &v,  4);
  memcpy(&p[15], &om, 4);
  p[19] = flags;
  p[20] = crc8(p, 20);
  if (Serial.availableForWrite() >= 21) Serial.write(p, 21);
}

// ---------- Controle ----------
enum Modo { PARADO, IR_PARA, AGUARDAR_TOPO, SWINGUP, EQUILIBRIO, RL };
Modo modo = PARADO;
Modo aposCentro = PARADO;
bool usarSwingup = true;
bool rlAposCaptura = false;        // 'r': ao capturar, entrega ao agente
float termoI = 0;
uint32_t chegouCentroEm = 0;

float vMs = 0;
long alvoPassos = 0;
bool zeroDefinido = false;
uint32_t tCtl = 0;
int rlCont = 0;

void rampaPara(float vDes, float aMax, float dt) {
  float dv = vDes - vMs;
  float passo = aMax * dt;
  if (dv > passo) dv = passo;
  else if (dv < -passo) dv = -passo;
  vMs += dv;
}

void mostrarPosicao() {
  if (modoBinario) return;
  float x_mm = (posicao - limiteB / 2) * 1000.0f / passosPorM;
  Serial.printf("Posicao: %ld passos (%.0f mm do centro) | Angulo: %.1f graus\n",
                (long)posicao, x_mm, anguloGraus());
}

void entrarRL() {
  modo = RL;
  modoBinario = true;
  aRL = 0.0f; aRL_pendente = 0.0f; aRL_nova = false;
  rlCont = 0;
  pacotesPerdidos = 0;
  ultimaAcaoMs = millis();
}

void sairRLparaPID(const char* motivo) {
  modo = EQUILIBRIO;
  termoI = 0;
  modoBinario = false;
  Serial.printf("# SUPERVISOR: %s -- PID retomou o controle\n", motivo);
}

void tratarComandoRL(uint8_t cmd) {
  switch (cmd) {
    case CMD_PARAR:
      modo = PARADO; aposCentro = PARADO; termoI = 0;
      modoBinario = false;
      break;
    case CMD_PID:
      if (modo == RL) sairRLparaPID("pedido do PC");
      break;
    case CMD_RL:
      if (modo == EQUILIBRIO) entrarRL();
      break;
  }
}

void controle(float dt) {
  atualizarAngulo();

  float x = (posicao - limiteB / 2) / passosPorM;
  float vMax = V_MAX_PASSOS * fatorV / passosPorM;
  float a = 0;
  float integralAnterior = termoI;
  bool calculouPID = false;
  bool saturou = false;
  uint8_t flags = 0;
  if (zeroDefinido) flags |= F_ZERO_OK;

  // Seguranca: fim de curso fisico (ou fio solto, ja que e NF)
  if (fimDeCurso(PIN_FC_A) || fimDeCurso(PIN_FC_B)) {
    if (modo != PARADO && !modoBinario)
      Serial.println("# Fim de curso acionado! Parando.");
    modo = PARADO; aposCentro = PARADO;
    termoI = 0; vMs = 0; modoBinario = false;
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
        termoI = 0; vMs = 0;
        if (!modoBinario) Serial.println("# Chegou.");
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
        if (!modoBinario) Serial.println("# PID capturou o pendulo.");
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
        if (!modoBinario) Serial.println("# Capturado! Equilibrando...");
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
        rlAposCaptura = false;
        if (!modoBinario) Serial.println("# Perdeu equilibrio.");
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

      // Pedido de entrega ao agente: so quando a haste estiver bem estavel.
      // Entregar um pendulo ja caindo seria preparar o agente para falhar.
      if (rlAposCaptura &&
          fabsf(psi) < 3.0f * PI / 180.0f &&
          fabsf(psiDot) < 0.5f &&
          fabsf(x) < 0.05f) {
        rlAposCaptura = false;
        Serial.println("# Entregando o controle ao agente. Protocolo binario ON.");
        Serial.flush();
        entrarRL();
      }
      break;
    }

    // ---------------- MODO RL ----------------
    case RL: {
      float psi = wrapPi(PI - phi);
      float psiDot = -omega;
      flags |= F_RL;

      // --- SUPERVISOR: o PID retoma antes de ser tarde ---
      if (supervisorLigado && fabsf(psi) > LIMITE_RL_GRAUS * PI / 180.0f) {
        sairRLparaPID("angulo acima do limite do agente");
        break;
      }
      if (fabsf(x) > X_LIM_RL) {
        sairRLparaPID("carrinho fora da faixa util");
        break;
      }

      // --- WATCHDOG: o PC sumiu ---
      if ((uint32_t)(millis() - ultimaAcaoMs) > RL_TIMEOUT_MS) {
        if (supervisorLigado) sairRLparaPID("comunicacao perdida");
        else { modo = PARADO; modoBinario = false;
               Serial.println("# RL: comunicacao perdida. Parado."); }
        break;
      }

      // --- fronteira de 20 ms: publica estado e troca a acao ---
      if (++rlCont >= RL_DIVISOR) {
        rlCont = 0;
        // A acao recebida durante os ultimos 20 ms so passa a valer AGORA.
        // Isso reproduz exatamente o atraso de 1 passo que o simulador
        // modelou. Aplicar assim que chega criaria um sistema mais rapido
        // do que aquele em que o agente treinou.
        if (aRL_nova) { aRL = aRL_pendente; aRL_nova = false; }
        else { pacotesPerdidos++; flags |= F_PERDEU; }
        enviarEstado(flags);
      }

      a = limitar(aRL, -A_BAL_MAX, A_BAL_MAX);
      break;
    }
  }

  // ---- integracao da velocidade, saturacoes e freio de parede ----
  // Vale igual para SWINGUP, EQUILIBRIO e RL: o agente nao escapa disso.
  if (modo == SWINGUP || modo == EQUILIBRIO || modo == RL) {
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
}

void tarefaControle() {
  uint32_t agora = micros();
  if ((uint32_t)(agora - tCtl) < 1000) return;   // 1 kHz
  float dt = (uint32_t)(agora - tCtl) * 1e-6f;
  tCtl = agora;
  if (dt > 0.010f) {
    modo = PARADO; aposCentro = PARADO;
    termoI = 0; vMs = 0; modoBinario = false;
    aplicarVelocidade(0);
    Serial.println("# Controle atrasou mais de 10 ms. Parado; reinicie com i/e.");
    return;
  }
  controle(dt);
}

void imprimirConstantes() {
  Serial.println("# pendulo_rl v1");
  Serial.printf("# passos_por_m=%.1f\n", passosPorM);
  Serial.printf("# curso_passos=%ld\n", limiteB);
  Serial.printf("# curso_m=%.4f\n", limiteB / passosPorM);
  Serial.printf("# curso_real_medido=%s\n", CURSO_REAL_M > 0 ? "sim" : "nao");
  Serial.printf("# l_eq_m=%.4f\n", L_EQ_M);
  Serial.printf("# contagens_por_volta=%d\n", CONTAGENS_POR_VOLTA);
  Serial.printf("# v_max_ms=%.4f (fator %.0f%%)\n",
                V_MAX_PASSOS * fatorV / passosPorM, 100 * fatorV);
  Serial.printf("# a_bal_max=%.2f\n", A_BAL_MAX);
  Serial.printf("# rl_hz=%d limite_rl_graus=%.1f timeout_ms=%u\n",
                1000 / RL_DIVISOR, LIMITE_RL_GRAUS, (unsigned)RL_TIMEOUT_MS);
  Serial.printf("# supervisor=%s\n", supervisorLigado ? "LIGADO" : "desligado");
  Serial.println("# psi = wrap(pi - phi)   psiDot = -omega");
}

void tratarComando(char c) {
  switch (tolower(c)) {
    case 'i':
    case 'e':
      if (!zeroDefinido) { Serial.println("# Envie z primeiro (haste pendurada e parada)."); break; }
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("# Pare com s antes."); break; }
      iniciarHistorico(); termoI = 0;
      usarSwingup = (tolower(c) == 'i');
      rlAposCaptura = false;
      alvoPassos = limiteB / 2; aposCentro = AGUARDAR_TOPO; modo = IR_PARA;
      Serial.println(usarSwingup ? "# Swing-up + PID." : "# Eleve a haste; o PID captura.");
      break;

    case 'r':
      if (!zeroDefinido) { Serial.println("# Envie z primeiro."); break; }
      if (modo == RL) { Serial.println("# Ja esta no modo RL."); break; }
      if (modo == EQUILIBRIO) {
        rlAposCaptura = true;
        Serial.println("# Vou entregar ao agente assim que a haste estabilizar.");
      } else {
        if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("# Pare com s antes."); break; }
        iniciarHistorico(); termoI = 0;
        usarSwingup = false; rlAposCaptura = true;
        alvoPassos = limiteB / 2; aposCentro = AGUARDAR_TOPO; modo = IR_PARA;
        Serial.println("# Centralizando. Eleve a haste; o PID captura e entrega ao agente.");
      }
      break;

    case 's':
      modo = PARADO; aposCentro = PARADO; termoI = 0;
      rlAposCaptura = false; modoBinario = false;
      Serial.println("# Parando.");
      break;

    case 'a': case 'b': case 'c':
      termoI = 0; aposCentro = PARADO; rlAposCaptura = false;
      alvoPassos = tolower(c) == 'a' ? 0 : (tolower(c) == 'b' ? limiteB : limiteB / 2);
      modo = IR_PARA;
      break;

    case 'z':
      if (modo != PARADO || fabsf(vMs) > 0.001f) { Serial.println("# Pare antes de zerar."); break; }
      zerarEncoder(); iniciarHistorico(); zeroDefinido = true;
      Serial.println("# Zero definido: pendulo pendurado = 0 graus.");
      break;

    case 'm':
      supervisorLigado = !supervisorLigado;
      Serial.printf("# Supervisor PID: %s\n", supervisorLigado ? "LIGADO" : "DESLIGADO");
      if (!supervisorLigado)
        Serial.println("# ATENCAO: sem supervisor, o agente perde a haste sem rede de protecao.");
      break;

    case 'v':
      fatorV = (fatorV < 0.6f) ? 0.75f : (fatorV < 0.9f ? 1.0f : 0.5f);
      Serial.printf("# Limitador de velocidade: %.0f%% (%.3f m/s)\n",
                    100 * fatorV, V_MAX_PASSOS * fatorV / passosPorM);
      break;

    case 'p': mostrarPosicao(); break;
    case 'k': imprimirConstantes(); break;
    case '\n': case '\r': case ' ': break;
    default:
      Serial.println("# i e r s a b c z p k | m=supervisor v=limite de velocidade");
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

  if (fimDeCurso(PIN_FC_A) && fimDeCurso(PIN_FC_B))
    parar("os dois fins de curso estao acionados (fio solto ou logica errada)");

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
  Serial.printf("# Limitador de velocidade em %.0f%% (comando v para mudar)\n", 100 * fatorV);
  Serial.println("# Indo ao meio. Com a haste pendurada e parada, envie z.");
  Serial.println("# i = swing-up+PID | e = PID manual | r = modo AGENTE | s = parar");
}

// ===================== LOOP =====================
void loop() {
  if (modoBinario) {
    processarSerialRL();     // protocolo binario; nada de texto
  } else if (Serial.available()) {
    tratarComando(Serial.read());
  }

  tarefaControle();

  // telemetria legivel, so fora do modo binario
  static uint32_t tPrint = 0;
  if (!modoBinario && (modo == SWINGUP || modo == EQUILIBRIO) &&
      millis() - tPrint > 200) {
    tPrint = millis();
    float x_mm = (posicao - limiteB / 2) * 1000.0f / passosPorM;
    char linha[110];
    int n = snprintf(linha, sizeof(linha), "%s ang=%.1f x=%.0fmm v=%.3f I=%.3f\n",
      modo == SWINGUP ? "SWING" : "PID", anguloGraus(), x_mm, vMs, termoI);
    if (n > 0 && n < (int)sizeof(linha) && Serial.availableForWrite() >= n)
      Serial.write((const uint8_t*)linha, n);
  }
}
