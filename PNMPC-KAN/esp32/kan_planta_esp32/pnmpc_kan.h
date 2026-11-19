// pnmpc_kan.h - PNMPC iterativo com a KAN como modelo de predição (PNMPCI-KAN).
// C++ puro: usado pelo sketch do ESP32 e pelo teste no PC. Inclua kan_eval.h antes.
//
// Estrutura (igual à do PNMPCI-ESNI, com a KAN no lugar do ESNI):
//   1) inovação   n(k) = y(k) - y_n(k), y_n(k) = KAN(s(k-1), u(k-1))  (filtrada; x e theta)
//   2) laço iterativo (iters vezes), a partir da sequência base u_base (warm start):
//        resposta livre  O_base = saídas previstas pela KAN com u_base (+ n)
//        linearização    G = dO/du por perturbação de cada movimento de u_base
//        QP              min |r - O_base - G du|^2_W + |D(u_base+du) - d0|^2_Rdu + |u_base+du|^2_Ru
//                        s.a. u_min <= u_base + du <= u_max        (Gauss-Seidel projetado)
//        u_base += du
//   3) aplica u(k) = u_base[0] (horizonte deslizante)
//
// Planta da bancada: estado s = [x v theta omega], theta = 0 pendurado, pi = em pé;
// entrada u = a_cmd [m/s^2]. Saídas do custo (calculadas do estado previsto):
//   o = [x, v, e_th = wrap(theta - theta_ref), omega, E, x_exc]
//   E = 0.5*omega^2/w0q - cos(theta)   (energia: -1 pendurado parado, +1 em pé parado)
//   x_exc = max(0, |x| - x_soft)       (penalidade suave do fim do trilho)
//   v_exc = max(0, |v| - v_soft)       (penalidade suave de velocidade: evita saturar o motor)
// Dois conjuntos de pesos (com histerese no erro de ângulo MEDIDO):
//   SWING-UP   (|e_th| > phi_sw): persegue E -> E_alvo, segura o carrinho no trilho
//   EQUILÍBRIO (|e_th| < phi_sw): persegue theta_ref, r_x, v = 0, omega = 0
#pragma once
#include <math.h>
#include <string.h>
#if defined(ESP_PLATFORM)
  #pragma GCC optimize ("O3")
#endif

#ifndef MPC_N
  #define MPC_N 40                        // horizonte de predição [passos de KAN_TS]
#endif
#define MPC_NU 4                          // movimentos de controle (blocagem)
#define MPC_NO 7                          // saídas do custo
static const int MPC_BLOCO[MPC_NU] = {0, 5, 10, 20};   // passo em que cada movimento começa

struct MpcParam {
  float q[4];       // EQUILÍBRIO: pesos de [x v e_th omega]
  float q_term;     // multiplicador do peso no último passo
  float r_du;       // peso nos incrementos entre movimentos (inclui u(k-1) -> u_0)
  float r_u;        // peso na magnitude de u
  float u_min, u_max;
  float eps;        // perturbação para a linearização [m/s^2]
  float alfa_n;     // filtro da inovação
  int   iters;      // iterações do laço PNMPC
  int   varreduras; // varreduras do Gauss-Seidel do QP
  float q_E;        // SWING-UP: peso do erro de energia
  float E_alvo;     // energia alvo (1 = em pé parado)
  float w0q;        // omega0^2 do pêndulo [rad^2/s^2]
  float phi_sw;     // |e_th| [rad] para entrar no modo equilíbrio (sai com 1,5x)
  float q_sw_x;     // SWING-UP: peso de x
  float q_sw_v;     // SWING-UP: peso de v
  float q_xs;       // peso da penalidade de trilho
  float x_soft;     // |x| a partir do qual a penalidade age [m]
  float Pt[10];     // EQUILÍBRIO: custo terminal LQR e^T P e, e = [x-r_x, v, e_th, omega]
                    // (triangular superior de P: p11 p12 p13 p14 p22 p23 p24 p33 p34 p44)
  int   paralelo;   // 0: y_n(k) = KAN(medição k-1, u(k-1));  1: modelo PARALELO como na figura
                    //    (a KAN roda só com u(k), n(k) corrige as 4 saídas, resposta livre parte do modelo)
  float v_max;      // velocidade máxima do carrinho [m/s] (motor de passo); 0 = sem limite.
                    // Na previsão, a aceleração que empurraria |v| além disso vira 0 (como no firmware).
  float q_vs;       // peso da penalidade suave de velocidade (evita a saturação do motor de passo)
  float v_soft;     // |v| a partir do qual a penalidade age [m/s] (0 = desligada)
};

// Padrões (o MATLAB manda os dele com o comando P no início do ensaio).
static MpcParam MPC = { {300.0f, 20.0f, 120.0f, 0.5f}, 3.0f, 5.0f, 0.01f, -5.0f, 5.0f, 0.05f, 0.7f, 1, 30,
                        100.0f, 1.1f, 43.67f, 0.5236f, 300.0f, 50.0f, 2000.0f, 0.12f,
                        {21033.5f, 6293.67f, -23284.0f, -3541.28f, 3269.99f, -12719.4f, -1941.24f, 60253.3f, 8975.97f, 1362.97f}, 0, 0.0f, 0.0f, 0.0f };   // Pt: pnmpc_lqr_terminal(kan_volta)
#define MPC_N_PAR 35

static inline void mpc_param_de_vetor(const float* p) {
  for (int i = 0; i < 4; i++) MPC.q[i] = p[i];
  MPC.q_term = p[4]; MPC.r_du = p[5]; MPC.r_u = p[6]; MPC.u_min = p[7]; MPC.u_max = p[8];
  MPC.eps = p[9]; MPC.alfa_n = p[10]; MPC.iters = (int)p[11]; MPC.varreduras = (int)p[12];
  MPC.q_E = p[13]; MPC.E_alvo = p[14]; MPC.w0q = p[15]; MPC.phi_sw = p[16];
  MPC.q_sw_x = p[17]; MPC.q_sw_v = p[18]; MPC.q_xs = p[19]; MPC.x_soft = p[20];
  for (int i = 0; i < 10; i++) MPC.Pt[i] = p[21 + i];
  MPC.paralelo = (int)p[31];
  MPC.v_max = p[32];
  MPC.q_vs = p[33]; MPC.v_soft = p[34];
}

struct MpcEstado {
  float ub[MPC_NU];   // sequência base (warm start)
  float u_ant;        // u aplicado no passo anterior
  float nf[4];        // inovação filtrada
  float pred[4];      // y_n(k+1) = KAN(s(k), u(k))
  bool  tem_pred;
  bool  equilibrio;   // modo atual
  float sm[4];        // estado do modelo paralelo (paralelo = 1)
  bool  tem_sm;
  int   chute;        // contador do chute inicial (pêndulo parado embaixo)
};
static MpcEstado MS;

struct MpcInfo { float u; float custo; float n_x; float n_th; int modo; float E; };

static inline void mpc_reiniciar() { memset(&MS, 0, sizeof(MS)); }

static inline float mpc_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static inline float mpc_u_em(const float* useq, int i) {
  int j = 0;
  for (int b = 1; b < MPC_NU; b++) if (i >= MPC_BLOCO[b]) j = b;
  return useq[j];
}

static inline float mpc_energia(float th, float w) { return 0.5f * w * w / MPC.w0q - cosf(th); }

// saídas do custo a partir de um estado (sem a inovação)
static inline void mpc_saidas(const float s[4], float th_ref, float o[MPC_NO]) {
  o[0] = s[0];
  o[1] = s[1];
  o[2] = kan_wrap_pi(s[2] - th_ref);
  o[3] = s[3];
  o[4] = mpc_energia(s[2], s[3]);
  const float ax = fabsf(s[0]) - MPC.x_soft;
  o[5] = ax > 0.0f ? ax : 0.0f;
  const float av = (MPC.v_soft > 0.0f) ? fabsf(s[1]) - MPC.v_soft : 0.0f;
  o[6] = av > 0.0f ? av : 0.0f;
}

// estados previstos X[i] (em k+1+i), i = i0..MPC_N-1, partindo de s0 (estado em k+i0)
static inline void mpc_rollout(const float s0[4], const float* useq, int i0, float X[][4]) {
  float s[4] = { s0[0], s0[1], s0[2], s0[3] };
  for (int i = i0; i < MPC_N; i++) {
    float a = mpc_u_em(useq, i);
    if (MPC.v_max > 0.0f && ((s[1] >= MPC.v_max && a > 0.0f) || (s[1] <= -MPC.v_max && a < 0.0f))) a = 0.0f;
    kan_prever(s, a, X[i]);
    if (MPC.v_max > 0.0f) X[i][1] = mpc_clamp(X[i][1], -MPC.v_max, MPC.v_max);
    for (int c = 0; c < 4; c++) s[c] = X[i][c];
  }
}

static float pnmpc_passo(const float s[4], float r_x, float th_ref, MpcInfo* info) {
  static float Xb[MPC_N][4], Xp[MPC_N][4], Ob[MPC_N][MPC_NO], G[MPC_N][MPC_NO][MPC_NU];
  const MpcParam& P = MPC;

  // ---- modo (histerese no erro de ângulo medido) ----
  const float eth_med = fabsf(kan_wrap_pi(s[2] - th_ref));
  if (MS.equilibrio) { if (eth_med > 1.5f * P.phi_sw) MS.equilibrio = false; }
  else               { if (eth_med < P.phi_sw) MS.equilibrio = true; }
  float W[MPC_NO];
  if (MS.equilibrio) { W[0] = P.q[0]; W[1] = P.q[1]; W[2] = P.q[2]; W[3] = P.q[3]; W[4] = 0.0f; }
  else               { W[0] = P.q_sw_x; W[1] = P.q_sw_v; W[2] = 0.0f; W[3] = 0.0f; W[4] = P.q_E; }
  W[5] = P.q_xs;
  W[6] = P.q_vs;
  const float ref[MPC_NO] = { r_x, 0.0f, 0.0f, 0.0f, P.E_alvo, 0.0f, 0.0f };

  // ---- 1) inovação n(k) = y(k) - y_n(k) ----
  float n[MPC_NO] = {0};
  const float* s0 = s;                       // estado de onde parte a resposta livre
  if (P.paralelo) {
    // como na figura: modelo em PARALELO com o processo (só recebe u); y_n(k) = estado do modelo
    if (!MS.tem_sm) { for (int c = 0; c < 4; c++) MS.sm[c] = s[c]; MS.tem_sm = true; }
    for (int c = 0; c < 4; c++) {
      float d = s[c] - MS.sm[c];
      if (c == 2) d = kan_wrap_pi(d);
      MS.nf[c] = P.alfa_n * MS.nf[c] + (1.0f - P.alfa_n) * d;
      n[c] = MS.nf[c];
    }
    s0 = MS.sm;
  } else {
    // y_n(k) = KAN(medição em k-1, u(k-1)); corrige x e theta
    if (MS.tem_pred) {
      const float nx = s[0] - MS.pred[0];
      const float nth = kan_wrap_pi(s[2] - MS.pred[2]);
      MS.nf[0] = P.alfa_n * MS.nf[0] + (1.0f - P.alfa_n) * nx;
      MS.nf[2] = P.alfa_n * MS.nf[2] + (1.0f - P.alfa_n) * nth;
    }
    n[0] = MS.nf[0]; n[2] = MS.nf[2];
  }

  float ub[MPC_NU];
  for (int j = 0; j < MPC_NU; j++) ub[j] = mpc_clamp(MS.ub[j], P.u_min, P.u_max);
  float custo = 0.0f;
  const float inv_eps = 1.0f / P.eps;

  // ---- 2) laço iterativo ----
  for (int it = 0; it < P.iters; it++) {
    mpc_rollout(s0, ub, 0, Xb);                                   // resposta livre
    for (int i = 0; i < MPC_N; i++) mpc_saidas(Xb[i], th_ref, Ob[i]);
    for (int j = 0; j < MPC_NU; j++) {                           // linearização
      float up[MPC_NU];
      for (int q = 0; q < MPC_NU; q++) up[q] = ub[q];
      up[j] += P.eps;
      const int i0 = MPC_BLOCO[j];
      mpc_rollout(i0 == 0 ? s0 : Xb[i0 - 1], up, i0, Xp);
      for (int i = 0; i < MPC_N; i++) {
        if (i < i0) { for (int c = 0; c < MPC_NO; c++) G[i][c][j] = 0.0f; continue; }
        float op[MPC_NO];
        mpc_saidas(Xp[i], th_ref, op);
        for (int c = 0; c < MPC_NO; c++) {
          float d = op[c] - Ob[i][c];
          if (c == 2) d = kan_wrap_pi(d);
          G[i][c][j] = d * inv_eps;
        }
      }
    }
    // QP: H du = -f
    float H[MPC_NU][MPC_NU] = {{0}}, f[MPC_NU] = {0};
    custo = 0.0f;
    for (int i = 0; i < MPC_N; i++) {
      const float mult = (i == MPC_N - 1 && !MS.equilibrio) ? P.q_term : 1.0f;   // no equilíbrio o terminal é o LQR
      for (int c = 0; c < MPC_NO; c++) {
        const float w = W[c] * mult;
        if (w == 0.0f) continue;
        float e = ref[c] - (Ob[i][c] + n[c]);
        if (c == 2) e = kan_wrap_pi(e);
        custo += w * e * e;
        for (int j = 0; j < MPC_NU; j++) {
          const float gj = G[i][c][j];
          if (gj == 0.0f) continue;
          f[j] -= w * gj * e;
          for (int l = 0; l < MPC_NU; l++) H[j][l] += w * gj * G[i][c][l];
        }
      }
    }
    // custo terminal LQR (equilíbrio): e^T P e no último passo, com P da Riccati da KAN linearizada
    if (MS.equilibrio) {
      static const int IP[4][4] = {{0,1,2,3},{1,4,5,6},{2,5,7,8},{3,6,8,9}};
      const int iN = MPC_N - 1;
      float eT[4];
      for (int a = 0; a < 4; a++) {
        eT[a] = ref[a] - (Ob[iN][a] + n[a]);
        if (a == 2) eT[a] = kan_wrap_pi(eT[a]);
      }
      for (int a = 0; a < 4; a++)
        for (int b = 0; b < 4; b++) {
          const float pab = P.Pt[IP[a][b]];
          if (pab == 0.0f) continue;
          custo += eT[a] * pab * eT[b];
          for (int j = 0; j < MPC_NU; j++) {
            f[j] -= G[iN][a][j] * pab * eT[b];
            for (int l = 0; l < MPC_NU; l++) H[j][l] += G[iN][a][j] * pab * G[iN][b][l];
          }
        }
    }
    float d[MPC_NU];
    for (int j = 0; j < MPC_NU; j++) d[j] = ub[j] - (j == 0 ? MS.u_ant : ub[j - 1]);
    for (int j = 0; j < MPC_NU; j++) {
      custo += P.r_du * d[j] * d[j] + P.r_u * ub[j] * ub[j];
      H[j][j] += P.r_du * (j < MPC_NU - 1 ? 2.0f : 1.0f) + P.r_u;
      if (j > 0) { H[j][j - 1] -= P.r_du; H[j - 1][j] -= P.r_du; }
      f[j] += P.r_du * (d[j] - (j < MPC_NU - 1 ? d[j + 1] : 0.0f)) + P.r_u * ub[j];
    }
    float du[MPC_NU] = {0}, invH[MPC_NU];
    for (int j = 0; j < MPC_NU; j++) invH[j] = H[j][j] > 0.0f ? 1.0f / H[j][j] : 0.0f;
    for (int v = 0; v < P.varreduras; v++) {
      for (int j = 0; j < MPC_NU; j++) {
        if (invH[j] == 0.0f) continue;
        float gr = f[j];
        for (int l = 0; l < MPC_NU; l++) gr += H[j][l] * du[l];
        du[j] = mpc_clamp(du[j] - gr * invH[j], P.u_min - ub[j], P.u_max - ub[j]);
      }
    }
    for (int j = 0; j < MPC_NU; j++) ub[j] += du[j];
  }

  // ---- chute inicial: parado embaixo, dE/du = 0 e o PNMPC não sai do lugar ----
  // onda quadrada de ±1 m/s^2 com meio período de 48 passos (0,48 s: ressonância, T0 = 2*pi/sqrt(w0q)), com o
  // PRIMEIRO pulso de meia largura (velocidade média zero: o carrinho oscila em vez de derivar),
  // até a energia passar de -0,95; depois o PNMPC assume.
  if (!MS.equilibrio && mpc_energia(s[2], s[3]) < -0.95f) {
    const int c = MS.chute++ + 24;                 // +24: primeiro pulso com meia largura
    ub[0] = ((c / 48) % 2) ? -1.0f : 1.0f;
    for (int j = 1; j < MPC_NU; j++) ub[j] = 0.0f;
  } else {
    MS.chute = 0;
  }

  // ---- 3) aplica o primeiro movimento ----
  const float u = ub[0];
  for (int j = 0; j < MPC_NU; j++) MS.ub[j] = ub[j];
  MS.u_ant = u;
  if (P.paralelo) {                // avança o modelo paralelo com o u aplicado
    float prox[4];
    kan_prever(MS.sm, u, prox);
    for (int c = 0; c < 4; c++) MS.sm[c] = isfinite(prox[c]) ? prox[c] : s[c];
  } else {
    kan_prever(s, u, MS.pred);      // y_n(k+1) para a inovação do próximo passo
    MS.tem_pred = true;
  }

  if (info) {
    info->u = u; info->custo = custo; info->n_x = n[0]; info->n_th = n[2];
    info->modo = MS.equilibrio ? 1 : 0; info->E = mpc_energia(s[2], s[3]);
  }
  return u;
}
