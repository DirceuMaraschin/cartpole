/*
  kan_planta_esp32.ino - Identificação da planta (carrinho + pêndulo) com KAN no ESP32, em HIL

  O MATLAB simula a planta em malha aberta (trem de pulsos na força) e, a cada Ts,
  manda o estado medido e a força aplicada. O ESP32 prevê, com a KAN, o estado do
  próximo instante, que o MATLAB compara com o que a planta realmente fez.

  Divisão dos núcleos (FreeRTOS):
    - núcleo 1 (APP_CPU): tarefa "KAN", dedicada só à rede (avaliação da KAN)
    - núcleo 0 (PRO_CPU): tarefa "Serial", recebe/decodifica/responde as linhas
    As duas conversam por filas (xQueue). O loop() do Arduino é apagado.

  Duas previsões por amostra:
    - um passo à frente: KAN(estado REAL da planta em k, u(k))      -> s1(k+1)
    - simulação livre:   KAN(estado PREVISTO pela própria KAN, u(k)) -> sl(k+1)
      (a KAN roda como um "gêmeo" da planta, sem ver a planta depois do R)

  CONTROLADOR PNMPC-KAN (pnmpc_kan.h), também no núcleo 1: o MATLAB (pnmpc_hil.m) simula
  a planta da bancada, manda a medição e a referência; o ESP32 devolve a_cmd.

  Protocolo (ASCII, uma linha por mensagem, terminada em '\n'):
    ?\n                     -> PRONTO,KAN_PLANTA_ESP32,1,<Ts>,<erro_autoteste>,<assinatura>,<nome>
    R,x,dx,th,dth\n         -> OK,<nucleo_da_KAN>   (reinicia a simulação livre nesse estado)
    S,k,u,x,dx,th,dth\n     -> P,k,x1,dx1,th1,dth1,xl,dxl,thl,dthl,t_us
         estado na convenção da KAN carregada; t_us = tempo das duas previsões [us]
    P,<21 parametros na ordem de mpc_param_de_vetor() em pnmpc_kan.h>\n
                            -> OK,P,<MPC_N>,<MPC_NU>          (sintonia do PNMPC)
    Z\n                     -> OK,Z                           (zera o PNMPC)
    C,k,x,v,th,w,r_x,r_th\n -> U,k,a_cmd,t_us,custo,n_x,n_th,modo,E   (um passo do PNMPC-KAN)
         th em rad na convenção da bancada (0 = pendurado, pi = em pé); modo 0 = swing-up, 1 = equilíbrio

  Os pesos vêm de kan_planta_pesos.h, gerado no MATLAB (ident_preparar_kan / ident_preparar_bancada).
  Placa: qualquer ESP32 de dois núcleos (ESP32, ESP32-S3). Core arduino-esp32 2.x ou 3.x.
*/

#include <Arduino.h>
#include "kan_planta_pesos.h"
#include "kan_eval.h"
#include "pnmpc_kan.h"

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2      // LED azul da maioria das placas ESP32 DevKit
#endif

#if CONFIG_FREERTOS_UNICORE
  #error "Esta placa tem um núcleo só (ESP32-S2/C3/C6). Use um ESP32 de dois núcleos."
#endif

// ===================== CONFIGURAÇÃO =====================
const uint32_t BAUD = 921600;        // igual a 'baud' em ident_hil_kan.m
const BaseType_t NUCLEO_KAN    = 1;  // núcleo dedicado à rede
const BaseType_t NUCLEO_SERIAL = 0;  // núcleo da comunicação

// ===================== FILAS E MENSAGENS =====================
struct Pedido {            // Serial -> KAN
  char     tipo;           // 'S' prever | 'R' reiniciar simulação livre | 'C' controle | 'P' sintonia | 'Z' zera PNMPC
  uint32_t k;
  float    u;              // força/entrada (S) ou referência r_x (C)
  float    s[4];
  float    par[MPC_N_PAR]; // sintonia (P)
  float    th_ref;         // referencia de angulo (C)
};

struct Resposta {          // KAN -> Serial
  char     tipo;
  uint32_t k;
  float    s1[4];          // um passo à frente
  float    sl[4];          // simulação livre
  uint32_t t_us;
  int      nucleo;
  MpcInfo  mpc;            // resultado do PNMPC (C)
};

static QueueHandle_t filaPedido;
static QueueHandle_t filaResposta;
static float erro_autoteste = NAN;

// ===================== TAREFA DA KAN (núcleo 1) =====================
static void tarefaKAN(void*) {
  float s_livre[4] = {0.0f, 0.0f, KAN_TH_EQ, 0.0f};   // estado da simulação livre
  Pedido p;
  Resposta r;
  for (;;) {
    if (xQueueReceive(filaPedido, &p, portMAX_DELAY) != pdTRUE) continue;
    r.tipo = p.tipo;
    r.k = p.k;
    r.nucleo = xPortGetCoreID();
    if (p.tipo == 'R') {
      for (int i = 0; i < 4; i++) { s_livre[i] = p.s[i]; r.s1[i] = r.sl[i] = p.s[i]; }
      r.t_us = 0;
    } else if (p.tipo == 'Z') {
      mpc_reiniciar();
      r.t_us = 0;
    } else if (p.tipo == 'P') {
      mpc_param_de_vetor(p.par);
      r.t_us = 0;
    } else if (p.tipo == 'C') {
      uint32_t t0 = micros();
      pnmpc_passo(p.s, p.u, p.th_ref, &r.mpc);   // PNMPC-KAN: a_cmd em r.mpc.u
      r.t_us = micros() - t0;
    } else {
      uint32_t t0 = micros();
      kan_prever(p.s, p.u, r.s1);               // um passo à frente (estado real)
      float prox[4];
      kan_prever(s_livre, p.u, prox);           // simulação livre (estado da KAN)
      for (int i = 0; i < 4; i++) {
        if (!isfinite(prox[i])) prox[i] = s_livre[i];   // não deixa NaN se propagar
        s_livre[i] = prox[i];
        r.sl[i] = prox[i];
      }
      r.t_us = micros() - t0;
    }
    xQueueSend(filaResposta, &r, portMAX_DELAY);
  }
}

// ===================== TAREFA DA SERIAL (núcleo 0) =====================
static int separar(char* linha, float* v, int max_v) {
  // linha = "X,a,b,c..." -> v = {a, b, c, ...}; devolve quantos números leu
  int n = 0;
  char* tok = strtok(linha, ",");      // pula a letra do comando
  tok = strtok(NULL, ",");
  while (tok != NULL && n < max_v) { v[n++] = strtof(tok, NULL); tok = strtok(NULL, ","); }
  return n;
}

static void processar_linha(char* linha) {
  char out[200];
  if (linha[0] == '?') {
    snprintf(out, sizeof(out), "PRONTO,KAN_PLANTA_ESP32,1,%.4f,%.3e,%.9e,%s", (double)KAN_TS,
             (double)erro_autoteste, (double)KAN_ASSINATURA, KAN_NOME);
    Serial.println(out);
    return;
  }

  Pedido p;
  float v[MPC_N_PAR];
  p.tipo = linha[0];
  if (p.tipo == 'Z') {
    p.k = 0; p.u = 0;
  } else if (p.tipo == 'P') {
    if (separar(linha, v, MPC_N_PAR) < MPC_N_PAR) { Serial.println("ERRO,campos_P"); return; }
    for (int i = 0; i < MPC_N_PAR; i++) p.par[i] = v[i];
    p.k = 0; p.u = 0;
  } else if (p.tipo == 'C') {
    if (separar(linha, v, 7) < 7) { Serial.println("ERRO,campos_C"); return; }
    p.k = (uint32_t)v[0];
    for (int i = 0; i < 4; i++) p.s[i] = v[1 + i];
    p.u = v[5];                                  // referencia r_x
    p.th_ref = v[6];                             // referencia de angulo
  } else if (p.tipo == 'R') {
    if (separar(linha, v, 4) < 4) { v[0] = 0; v[1] = 0; v[2] = KAN_TH_EQ; v[3] = 0; }
    p.k = 0; p.u = 0;
    for (int i = 0; i < 4; i++) p.s[i] = v[i];
  } else if (p.tipo == 'S') {
    if (separar(linha, v, 6) < 6) { Serial.println("ERRO,campos"); return; }
    p.k = (uint32_t)v[0];
    p.u = v[1];
    for (int i = 0; i < 4; i++) p.s[i] = v[2 + i];
  } else {
    Serial.println("ERRO,comando");
    return;
  }

  xQueueSend(filaPedido, &p, portMAX_DELAY);
  Resposta r;
  if (xQueueReceive(filaResposta, &r, pdMS_TO_TICKS(200)) != pdTRUE) {
    Serial.println("ERRO,timeout_kan");
    return;
  }
  if (r.tipo == 'R') {
    snprintf(out, sizeof(out), "OK,%d", r.nucleo);
  } else if (r.tipo == 'Z') {
    snprintf(out, sizeof(out), "OK,Z");
  } else if (r.tipo == 'P') {
    snprintf(out, sizeof(out), "OK,P,%d,%d", MPC_N, MPC_NU);
  } else if (r.tipo == 'C') {
    snprintf(out, sizeof(out), "U,%lu,%.7g,%lu,%.6g,%.6g,%.6g,%d,%.5g", (unsigned long)r.k, (double)r.mpc.u,
             (unsigned long)r.t_us, (double)r.mpc.custo, (double)r.mpc.n_x, (double)r.mpc.n_th,
             r.mpc.modo, (double)r.mpc.E);
  } else {
    snprintf(out, sizeof(out), "P,%lu,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%.7g,%lu",
             (unsigned long)r.k,
             (double)r.s1[0], (double)r.s1[1], (double)r.s1[2], (double)r.s1[3],
             (double)r.sl[0], (double)r.sl[1], (double)r.sl[2], (double)r.sl[3],
             (unsigned long)r.t_us);
  }
  Serial.println(out);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

static void tarefaSerial(void*) {
  static char buf[400];
  size_t n = 0;
  for (;;) {
    bool leu = false;
    while (Serial.available() > 0) {
      leu = true;
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf[n] = '\0';
        if (n > 0) processar_linha(buf);
        n = 0;
      } else if (n < sizeof(buf) - 1) {
        buf[n++] = c;
      } else {
        n = 0;   // linha longa demais: descarta
      }
    }
    if (!leu) vTaskDelay(1);   // libera o núcleo 0 (watchdog/idle) quando não há dados
  }
}

// ===================== AUTOTESTE =====================
// Compara a previsão em float do ESP32 com a do MATLAB (double) gravada no .h
static float autoteste() {
  float erro = 0.0f;
  for (int t = 0; t < KAN_N_TESTE; t++) {
    float s1[4];
    kan_prever(KAN_TESTE_S[t], KAN_TESTE_U[t], s1);
    for (int i = 0; i < 4; i++) {
      float e = fabsf(s1[i] - KAN_TESTE_S1[t][i]);
      if (e > erro) erro = e;
    }
  }
  return erro;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.setRxBufferSize(1024);
  Serial.begin(BAUD);
  delay(200);
  erro_autoteste = autoteste();

  filaPedido   = xQueueCreate(4, sizeof(Pedido));
  filaResposta = xQueueCreate(4, sizeof(Resposta));
  xTaskCreatePinnedToCore(tarefaKAN,    "KAN",    8192, NULL, 5, NULL, NUCLEO_KAN);
  xTaskCreatePinnedToCore(tarefaSerial, "Serial", 6144, NULL, 4, NULL, NUCLEO_SERIAL);
}

void loop() {
  vTaskDelete(NULL);   // todo o trabalho fica nas duas tarefas acima
}
