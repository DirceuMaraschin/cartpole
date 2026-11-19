// kan_eval.h - avaliação da KAN (formato pykan) com várias saídas, em float.
// C++ puro (sem Arduino): usado pelo sketch do ESP32 e pelo teste no PC.
// Inclua "kan_planta_pesos.h" ANTES deste arquivo.
//
// Para cada camada e saída j (mesma conta de ident_kan_forward.m / pykan):
//   s_j = sum_i sb_ij*SiLU(x_i) + sp_ij*sum_m coef_ijm*B_m(x_i)
//   x_j = a_j*s_j + c_j
// B_m: B-splines de ordem KAN_K (Cox-de Boor) sobre o grid estendido de cada entrada.
#pragma once
#include <math.h>
#include <stdint.h>
#if defined(ESP_PLATFORM)
  #pragma GCC optimize ("O3")   // a IDE compila com -Os; a KAN roda centenas de vezes por amostra no PNMPC
#endif

// B-splines de ordem KAN_K no ponto x; g = grid estendido com ng pontos; B recebe ng-1 valores
// (só os ng-1-KAN_K primeiros são as bases finais).
static inline void kan_bsplines(float x, const float* g, int ng, float* B) {
  int n = ng - 1;
  for (int m = 0; m < n; m++) B[m] = (x >= g[m] && x < g[m + 1]) ? 1.0f : 0.0f;
  for (int p = 1; p <= KAN_K; p++) {
    n = ng - 1 - p;
    for (int m = 0; m < n; m++) {
      float d1 = g[m + p] - g[m], d2 = g[m + p + 1] - g[m + 1];
      float t1 = (d1 != 0.0f) ? (x - g[m]) / d1 * B[m] : 0.0f;
      float t2 = (d2 != 0.0f) ? (g[m + p + 1] - x) / d2 * B[m + 1] : 0.0f;
      B[m] = t1 + t2;   // B[m+1] ainda é o valor antigo: pode sobrescrever no lugar
    }
  }
}

// Caminho rápido para grid UNIFORME e k = 3 (o caso do treino no MATLAB): só 4 bases
// são não nulas em cada ponto e têm forma fechada -> sem as divisões do Cox-de Boor
// (o Xtensa do ESP32 não tem divisão de float em hardware). Mesmo resultado do Cox-de Boor.
// Devolve false se x está fora do grid (todas as bases = 0).
static inline bool kan_bspline3_unif(float x, const float* g, float inv_h, int ng, int* m0, float b[4]) {
  const float t = (x - g[0]) * inv_h;          // inv_h = 1/(g[1]-g[0]), calculado uma vez
  if (!(t >= 0.0f) || t >= (float)(ng - 1)) return false;
  const int j = (int)t;
  const float u = t - (float)j, u2 = u * u, u3 = u2 * u, w = 1.0f - u;
  const float s6 = 1.0f / 6.0f;
  b[0] = w * w * w * s6;                                   // base j-3
  b[1] = (3.0f * u3 - 6.0f * u2 + 4.0f) * s6;              // base j-2
  b[2] = (-3.0f * u3 + 3.0f * u2 + 3.0f * u + 1.0f) * s6;  // base j-1
  b[3] = u3 * s6;                                          // base j
  *m0 = j - 3;
  return true;
}

#if defined(KAN_GRID_UNIFORME) && KAN_GRID_UNIFORME && KAN_K == 3
  #define KAN_RAPIDA 1
#else
  #define KAN_RAPIDA 0
#endif

// entrada: KAN_N_ENT valores normalizados; saida: KAN_N_SAI valores (antes de KAN_SAI_*).
// Reentrante (buffers na pilha): pode ser chamada de qualquer tarefa.
static inline void kan_avaliar(const float* entrada, float* saida) {
  float a[KAN_MAX_NEUR], s[KAN_MAX_NEUR];
#if !KAN_RAPIDA
  float B[KAN_MAX_GRID];
#endif
  for (int i = 0; i < KAN_TAM[0]; i++) a[i] = entrada[i];
#if KAN_RAPIDA
  // 1/(passo do grid) de cada entrada de cada camada: evita uma divisão por entrada
  static float inv_h[KAN_N_CAMADAS][KAN_MAX_NEUR];
  static bool inv_ok = false;
  if (!inv_ok) {
    const float* q = KAN_PESOS;
    for (int l = 0; l < KAN_N_CAMADAS; l++) {
      const int ne = KAN_TAM[l], ns = KAN_TAM[l + 1], ng = KAN_NG[l], nb = ng - KAN_K - 1;
      for (int i = 0; i < ne; i++) inv_h[l][i] = 1.0f / (q[i * ng + 1] - q[i * ng]);
      q += ne * ng + ne * ns * nb + 2 * ne * ns + 2 * ns;
    }
    inv_ok = true;
  }
#endif

  const float* p = KAN_PESOS;
  for (int l = 0; l < KAN_N_CAMADAS; l++) {
    const int ne = KAN_TAM[l], ns = KAN_TAM[l + 1], ng = KAN_NG[l], nb = ng - KAN_K - 1;
    const float* grid = p;
    const float* coef = grid + ne * ng;
    const float* sb   = coef + ne * ns * nb;
    const float* sp   = sb + ne * ns;
    const float* ka   = sp + ne * ns;
    const float* kc   = ka + ns;
    for (int j = 0; j < ns; j++) s[j] = 0.0f;
    for (int i = 0; i < ne; i++) {
      const float x = a[i];
      const float silu = x / (1.0f + expf(-x));
#if KAN_RAPIDA
      int m0; float b[4];
      const bool dentro = kan_bspline3_unif(x, grid + i * ng, inv_h[l][i], ng, &m0, b);
      for (int j = 0; j < ns; j++) {
        const float* cij = coef + (i * ns + j) * nb;
        float spl = 0.0f;
        if (dentro)
          for (int q = 0; q < 4; q++) {
            const int m = m0 + q;
            if (m >= 0 && m < nb) spl += cij[m] * b[q];
          }
        s[j] += sb[i * ns + j] * silu + sp[i * ns + j] * spl;
      }
#else
      kan_bsplines(x, grid + i * ng, ng, B);
      for (int j = 0; j < ns; j++) {
        const float* cij = coef + (i * ns + j) * nb;
        float spl = 0.0f;
        for (int m = 0; m < nb; m++) spl += cij[m] * B[m];
        s[j] += sb[i * ns + j] * silu + sp[i * ns + j] * spl;
      }
#endif
    }
    for (int j = 0; j < ns; j++) a[j] = ka[j] * s[j] + kc[j];
    p = kc + ns;
  }
  for (int j = 0; j < KAN_TAM[KAN_N_CAMADAS]; j++) saida[j] = a[j];
}

// wrap em [-pi, pi): igual a ident_regressores.m
static inline float kan_wrap_pi(float a) {
  const float PI_F = 3.14159265358979f;
  return a - 2.0f * PI_F * floorf((a + PI_F) / (2.0f * PI_F));
}

// Previsão um passo à frente da planta:
//   s = [x dx th dth] em k, u = força em [k, k+1)  ->  s1 = estado previsto em k+1
// Regressores: KAN_N_ENT == 4 -> [v, phi, w, u]; KAN_N_ENT == 5 -> [v, sin phi, cos phi, w, u]
// (phi = wrap(th - KAN_TH_EQ); a versão sin/cos vale para a volta inteira do pêndulo)
static inline void kan_prever(const float s[4], float u, float s1[4]) {
  const float phi = kan_wrap_pi(s[2] - KAN_TH_EQ);
#if KAN_N_ENT == 5
  const float reg[5] = { s[1], sinf(phi), cosf(phi), s[3], u };
#elif KAN_N_ENT == 4
  const float reg[4] = { s[1], phi, s[3], u };
#else
  #error "kan_prever: a KAN da planta deve ter 4 ou 5 entradas"
#endif
  float xn[KAN_N_ENT], y[KAN_N_SAI];
  for (int i = 0; i < KAN_N_ENT; i++) xn[i] = KAN_ENT_GANHO[i] * reg[i] + KAN_ENT_DESLOC[i];
  kan_avaliar(xn, y);
  for (int j = 0; j < 4; j++) {
    // KAN residual: parte linear [regressores 1]*KAN_LIN somada à saída da KAN
    float lin = KAN_LIN[KAN_N_ENT][j];
    for (int i = 0; i < KAN_N_ENT; i++) lin += reg[i] * KAN_LIN[i][j];
    s1[j] = s[j] + (KAN_SAI_GANHO[j] * y[j] + KAN_SAI_DESLOC[j]) + lin;
  }
}
