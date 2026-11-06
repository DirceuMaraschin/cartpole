#!/usr/bin/env python3
"""
Converte o Ator treinado (.pt) em C compilável para o ESP32-S3.

Gera quatro arquivos em --saida:

    politica.h        interface + constantes de normalização
    politica.c        pesos (const, em flash) + inferência
    oraculo.h         vetores de teste com a saída de referência do torch
    teste_politica.c  main() que roda o oráculo e mede o erro

O QUE SAI E O QUE FICA. Do checkpoint só o ATOR interessa: o crítico existe
para calcular vantagens durante o treino e o log_std para explorar. Em
execução a ação é a média determinística, então os 4 673 parâmetros do ator
são tudo. O crítico e o log_std são descartados aqui.

A ORDEM DOS PESOS É O CONTRATO. nn.Linear guarda weight como
[saídas][entradas], que é exatamente o laço natural em C:

    y[i] = b[i] + soma_j W[i][j] * x[j]

Nenhuma transposição. Transpor por engano não dá erro de compilação: dá um
agente que decide ao contrário, e isso só aparece na bancada.

POR QUE UM ORÁCULO. A troca torch -> numpy do bridge fechou em 2,4e-7 porque
havia com o que comparar. Aqui é a mesma disciplina: o C tem de reproduzir o
torch nos mesmos vetores antes de chegar perto da placa.

Uso:
    python exportar_c.py                                  # modelos_sw/melhor.pt
    python exportar_c.py modelos_sw/agente.pt --saida firmware/politica
"""

import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pendulo_env import X_LIM, V_MAX, A_MAX, DT_CTRL
from pendulo_env_sw import PSID_ESC as PSID_ESC_SW
from ppo import AtorCritico

DIM_OBS = 6
DIM_ACAO = 1
OCULTA = 64
CAMADAS = (0, 2, 4)          # os índices das nn.Linear dentro de ator (Sequential)

N_ORACULO = 256
LUT_N = 256                  # intervalos da tabela de tanh
LUT_LIM = 4.0                # tanh(4) = 0,999329; fora disso saturamos


# =====================================================================
# leitura do checkpoint
# =====================================================================
def ler_pesos(caminho):
    """Devolve [(W, b), ...] em float32, só do ator, na ordem das camadas."""
    est = torch.load(caminho, map_location='cpu', weights_only=True)['rede']
    camadas = []
    for i in CAMADAS:
        W = est[f'ator.{i}.weight'].numpy().astype(np.float32)
        b = est[f'ator.{i}.bias'].numpy().astype(np.float32)
        camadas.append((W, b))

    formas = [(c[0].shape, c[1].shape) for c in camadas]
    esperado = [((OCULTA, DIM_OBS), (OCULTA,)),
                ((OCULTA, OCULTA), (OCULTA,)),
                ((DIM_ACAO, OCULTA), (DIM_ACAO,))]
    if formas != esperado:
        raise SystemExit(f'Formas inesperadas no checkpoint:\n'
                         f'  lido:     {formas}\n'
                         f'  esperado: {esperado}')
    return camadas


def carregar_torch(caminho):
    """O ator em torch, para gerar a referência."""
    rede = AtorCritico(DIM_OBS, DIM_ACAO, oculta=OCULTA)
    rede.load_state_dict(torch.load(caminho, map_location='cpu',
                                    weights_only=True)['rede'])
    rede.eval()
    return rede


# =====================================================================
# formatação de float para C
# =====================================================================
def f32(v):
    """float32 em texto com ida-e-volta exata.

    9 dígitos significativos é o mínimo que garante reconstruir qualquer
    float32 sem perda. Menos que isso introduz um erro silencioso entre o
    .pt e o .c, e seria indistinguível de um bug de indexação.
    """
    x = float(v)
    if not np.isfinite(x):
        raise AssertionError(f'peso nao finito no checkpoint: {v!r}')
    t = f'{x:.9g}'
    if np.float32(t) != np.float32(x):
        raise AssertionError(f'ida-e-volta falhou para {v!r} -> {t}')
    # "0f" e "2f" sao sufixo invalido em constante inteira: o literal precisa
    # de ponto ou expoente para ser float.
    if not any(c in t for c in '.eE'):
        t += '.0'
    return t + 'f'


def vetor_c(v, por_linha=8, recuo='    '):
    itens = [f32(x) for x in np.asarray(v).ravel()]
    linhas = [recuo + ', '.join(itens[i:i + por_linha])
              for i in range(0, len(itens), por_linha)]
    return ',\n'.join(linhas)


def matriz_c(M, recuo='    '):
    """Row-major, uma linha de texto por linha da matriz quando couber."""
    partes = []
    for linha in M:
        itens = [f32(x) for x in linha]
        if len(itens) <= 8:
            partes.append(recuo + '{ ' + ', '.join(itens) + ' }')
        else:
            quebra = ',\n' + recuo + '  '
            grupos = [', '.join(itens[i:i + 8]) for i in range(0, len(itens), 8)]
            partes.append(recuo + '{ ' + quebra.join(grupos) + ' }')
    return ',\n'.join(partes)


# =====================================================================
# oráculo
# =====================================================================
def gerar_oraculo(rede, semente=20261105):
    """Vetores de teste: casos de borda primeiro, depois amostras do envelope.

    Os casos de borda vêm primeiro de propósito: se o C estiver errado, é
    neles que o erro é maior e o diagnóstico mais fácil de ler.

    As amostras aleatórias respeitam sin^2 + cos^2 = 1 (sorteio em psi, não
    em sin e cos separados), porque é o único lugar do espaço de entrada que
    o hardware pode produzir.
    """
    rng = np.random.default_rng(semente)
    obs = []

    # --- casos de borda ---
    obs.append([0, 0, 0, 1, 0, 0])                      # em pé, parado, centro
    obs.append([0, 0, 0, -1, 0, 0])                     # pendurado, parado
    for s in (-1, 1):
        obs.append([s, 0, 0, 1, 0, 0])                  # fim do trilho
        obs.append([0, s, 0, 1, 0, 0])                  # velocidade máxima
        obs.append([0, 0, 0, 1, s, 0])                  # omega máximo
        obs.append([0, 0, 0, 1, 0, s])                  # ação anterior saturada
        obs.append([s, s, 0, 1, s, s])                  # tudo saturado junto
        obs.append([s, -s, 0, -1, -s, s])
    for psi in (-np.pi, -np.pi / 2, -0.1, 0.0, 0.1, np.pi / 2, np.pi):
        obs.append([0, 0, np.sin(psi), np.cos(psi), 0, 0])
    # fora do envelope: a rede recebe isso se o encoder escorregar
    obs.append([2.0, 2.0, 0.0, 1.0, 3.0, 2.0])
    obs.append([-2.0, -2.0, 0.0, -1.0, -3.0, -2.0])

    # --- amostras do envelope ---
    while len(obs) < N_ORACULO:
        psi = rng.uniform(-np.pi, np.pi)
        obs.append([rng.uniform(-1.1, 1.1),             # x/X_LIM
                    rng.uniform(-1.1, 1.1),             # v/V_MAX
                    np.sin(psi), np.cos(psi),
                    rng.uniform(-1.1, 1.1),             # -omega/PSID_ESC
                    rng.uniform(-1.0, 1.0)])            # a_ant/A_MAX

    obs = np.asarray(obs[:N_ORACULO], dtype=np.float32)

    with torch.no_grad():
        ref32 = rede.ator(torch.from_numpy(obs)).numpy().ravel().astype(np.float32)

    # referência em float64, com os mesmos pesos: separa erro de
    # implementação (grande) de erro de precisão (~1e-7)
    Wb = [(W.astype(np.float64), b.astype(np.float64))
          for W, b in [(rede.ator[i].weight.detach().numpy(),
                        rede.ator[i].bias.detach().numpy()) for i in CAMADAS]]
    h = obs.astype(np.float64)
    h = np.tanh(h @ Wb[0][0].T + Wb[0][1])
    h = np.tanh(h @ Wb[1][0].T + Wb[1][1])
    ref64 = (h @ Wb[2][0].T + Wb[2][1]).ravel()

    return obs, ref32, ref64


# =====================================================================
# escrita dos arquivos
# =====================================================================
CABECALHO = """/* GERADO POR exportar_c.py -- NAO EDITAR A MAO.
 *
 * Origem : {origem}
 * Modelo : ator {dim_obs} -> {oculta} -> {oculta} -> {dim_acao}, tanh, cabeca linear
 * Pesos  : {n_par} parametros, {n_bytes} bytes em float32
 */
"""


def escrever_h(cam, destino, origem):
    n_par = sum(W.size + b.size for W, b in cam)
    n_bytes = 4 * n_par
    txt = CABECALHO.format(origem=origem, dim_obs=DIM_OBS, oculta=OCULTA,
                           dim_acao=DIM_ACAO, n_par=n_par, n_bytes=n_bytes)
    txt += f"""
#ifndef POLITICA_H
#define POLITICA_H

#ifdef __cplusplus
extern "C" {{
#endif

#define POLITICA_DIM_OBS   {DIM_OBS}
#define POLITICA_OCULTA    {OCULTA}
#define POLITICA_DIM_ACAO  {DIM_ACAO}
#define POLITICA_N_PARAM   {n_par}

/* Constantes de normalizacao -- geradas de pendulo_env.py e pendulo_env_sw.py.
 *
 * NAO as redigite em outro lugar do firmware. Se uma escala divergir entre o
 * PC e a placa, o agente passa a ver um mundo que nao e o dele e o sintoma
 * aparece so na bancada, como "o agente ficou pior".
 */
#define POLITICA_X_LIM       {X_LIM}f      /* m     limite de software do trilho */
#define POLITICA_V_MAX       {V_MAX}f      /* m/s   escala da velocidade         */
#define POLITICA_PSID_ESC    {PSID_ESC_SW}f     /* rad/s escala da velocidade angular */
#define POLITICA_A_MAX       {A_MAX}f      /* m/s2  saturacao da acao            */
#define POLITICA_DT          {DT_CTRL}f     /* s     passo do agente (50 Hz)      */

/* Ordem da observacao -- e um contrato com a rede treinada.
 *
 *   obs[0] =  x / X_LIM
 *   obs[1] =  v / V_MAX
 *   obs[2] =  sin(psi)
 *   obs[3] =  cos(psi)
 *   obs[4] = -omega / PSID_ESC
 *   obs[5] =  a_anterior / A_MAX
 *
 * psi = 0 com a haste EM PE. Trocar duas entradas de lugar nao da erro de
 * compilacao: da um agente que decide ao contrario.
 */
enum {{
    POL_X = 0, POL_V = 1, POL_SIN = 2, POL_COS = 3, POL_OMEGA = 4, POL_A_ANT = 5
}};

/* Copia os pesos de flash para SRAM interna. Chamar uma vez no setup().
 *
 * Por que copiar: const vive em .rodata, atras do cache de flash. Um miss no
 * meio da inferencia e jitter, e jitter num sistema que dobra o erro a cada
 * 105 ms nao e detalhe. Custa {n_bytes} bytes dos 512 KiB de SRAM.
 */
void politica_iniciar(void);

/* Saida crua da rede, em unidades normalizadas (tipicamente em [-1, 1]).  */
float politica_bruta(const float obs[POLITICA_DIM_OBS]);

/* Aceleracao em m/s2: clip(bruta, -1, +1) * A_MAX.
 * Identico a rl_bridge.py: a = clip(acao, -1, 1) * A_MAX.
 */
float politica_aceleracao(const float obs[POLITICA_DIM_OBS]);

/* Monta a observacao a partir do estado fisico, com as escalas acima.
 * psi em radianos, 0 = em pe. Devolve em obs[].
 */
void politica_observar(float x_m, float v_ms, float psi_rad, float omega_rads,
                       float a_ant_ms2, float obs[POLITICA_DIM_OBS]);

#ifdef __cplusplus
}}
#endif

#endif /* POLITICA_H */
"""
    with open(destino, 'w') as f:
        f.write(txt)
    return n_par


def escrever_c(cam, destino, origem):
    (W1, b1), (W2, b2), (W3, b3) = cam
    n_par = sum(W.size + b.size for W, b in cam)

    # tabela de tanh: LUT_N intervalos em [-LIM, +LIM], LUT_N+1 pontos
    xs = np.linspace(-LUT_LIM, LUT_LIM, LUT_N + 1)
    lut = np.tanh(xs).astype(np.float32)
    h = 2 * LUT_LIM / LUT_N
    # erro teorico da interpolacao linear: h^2/8 * max|tanh''| (= 0,7698)
    err_lut = h * h / 8 * 0.7698

    txt = CABECALHO.format(origem=origem, dim_obs=DIM_OBS, oculta=OCULTA,
                           dim_acao=DIM_ACAO, n_par=n_par, n_bytes=4 * n_par)
    txt += f"""
#include "politica.h"

#include <math.h>
#include <string.h>

/* O core do ESP32 compila com -Os (tamanho). Num laco denso de float isso
 * custa caro, e a IDE do Arduino nao expoe a opcao -- dai o pragma. Medido no
 * ESP32 Dev Module com -O2: 335 us de p99, sendo 70% em tanhf. Sem o pragma o
 * numero sai pessimista e da a impressao de que a LUT e necessaria.
 * Em quem ja passa -O2 na linha de comando, e inofensivo. */
#pragma GCC optimize ("O2")

/* LIGAR A TABELA DE TANH SEM MEXER NA LINHA DE COMANDO.
 * Descomente a linha abaixo -- util na IDE do Arduino, que nao da acesso
 * facil as flags do compilador. Leia o bloco seguinte antes. */
/* #define POLITICA_TANH_LUT */

/* ------------------------------------------------------------------ *
 * tanh
 *
 * O custo da inferencia NAO esta nas {W1.size + W2.size + W3.size} multiplicacoes: esta aqui. Sao
 * {2 * OCULTA} chamadas por passo, e tanhf() da libm e caro no Xtensa.
 *
 * Com POLITICA_TANH_LUT definido, troca-se por tabela de {LUT_N} intervalos em
 * [-{LUT_LIM:.0f}, +{LUT_LIM:.0f}] com interpolacao linear. Erro teorico {err_lut:.2e} por
 * ativacao, e a tabela custa {4 * (LUT_N + 1)} bytes.
 *
 * MEDIR ANTES DE LIGAR. E depois de ligar, validar em MALHA FECHADA: num
 * sistema que dobra o erro a cada 105 ms, 1e-4 de vies por ativacao nao e
 * obviamente inofensivo. O teste certo e rodar avaliar_sw.py com a
 * aproximacao no laco e conferir que frac_pe e psi_pe nao se movem.
 * ------------------------------------------------------------------ */
#ifdef POLITICA_TANH_LUT

#define LUT_N    {LUT_N}
#define LUT_LIM  {LUT_LIM:.1f}f
#define LUT_ESC  ({f32(LUT_N / (2 * LUT_LIM))})   /* LUT_N / (2*LUT_LIM) */

static const float TANH_LUT[LUT_N + 1] = {{
{vetor_c(lut)}
}};

static inline float t_anh(float x)
{{
    if (x <= -LUT_LIM) return -1.0f;
    if (x >=  LUT_LIM) return  1.0f;
    float u = (x + LUT_LIM) * LUT_ESC;
    int   i = (int)u;                  /* u >= 0, truncar basta */
    float f = u - (float)i;
    return TANH_LUT[i] + f * (TANH_LUT[i + 1] - TANH_LUT[i]);
}}

#else
static inline float t_anh(float x) {{ return tanhf(x); }}
#endif

/* ------------------------------------------------------------------ *
 * Pesos, em flash (.rodata)
 *
 * Row-major [saidas][entradas], a mesma ordem de nn.Linear.weight:
 *     y[i] = b[i] + soma_j W[i][j] * x[j]
 * ------------------------------------------------------------------ */
static const float F_W1[{OCULTA}][{DIM_OBS}] = {{
{matriz_c(W1)}
}};
static const float F_B1[{OCULTA}] = {{
{vetor_c(b1)}
}};

static const float F_W2[{OCULTA}][{OCULTA}] = {{
{matriz_c(W2)}
}};
static const float F_B2[{OCULTA}] = {{
{vetor_c(b2)}
}};

static const float F_W3[{DIM_ACAO}][{OCULTA}] = {{
{matriz_c(W3)}
}};
static const float F_B3[{DIM_ACAO}] = {{
{vetor_c(b3)}
}};

/* ------------------------------------------------------------------ *
 * Copia em SRAM interna
 * ------------------------------------------------------------------ */
static float W1_[{OCULTA}][{DIM_OBS}], B1_[{OCULTA}];
static float W2_[{OCULTA}][{OCULTA}],  B2_[{OCULTA}];
static float W3_[{DIM_ACAO}][{OCULTA}], B3_[{DIM_ACAO}];
static int   pronto = 0;

void politica_iniciar(void)
{{
    memcpy(W1_, F_W1, sizeof(W1_));  memcpy(B1_, F_B1, sizeof(B1_));
    memcpy(W2_, F_W2, sizeof(W2_));  memcpy(B2_, F_B2, sizeof(B2_));
    memcpy(W3_, F_W3, sizeof(W3_));  memcpy(B3_, F_B3, sizeof(B3_));
    pronto = 1;
}}

/* ------------------------------------------------------------------ *
 * Inferencia
 *
 * Tres produtos matriz-vetor e duas camadas de tanh. Se a medicao na placa
 * pedir mais velocidade, a troca natural e dsps_dotprod_f32() da ESP-DSP no
 * laco interno da segunda camada, que concentra {W2.size} das {W1.size + W2.size + W3.size} multiplicacoes.
 * ------------------------------------------------------------------ */
float politica_bruta(const float obs[POLITICA_DIM_OBS])
{{
    float h1[{OCULTA}], h2[{OCULTA}];
    int i, j;

    if (!pronto) politica_iniciar();

    for (i = 0; i < {OCULTA}; i++) {{
        float s = B1_[i];
        for (j = 0; j < {DIM_OBS}; j++) s += W1_[i][j] * obs[j];
        h1[i] = t_anh(s);
    }}

    for (i = 0; i < {OCULTA}; i++) {{
        float s = B2_[i];
        for (j = 0; j < {OCULTA}; j++) s += W2_[i][j] * h1[j];
        h2[i] = t_anh(s);
    }}

    float y = B3_[0];
    for (j = 0; j < {OCULTA}; j++) y += W3_[0][j] * h2[j];
    return y;
}}

float politica_aceleracao(const float obs[POLITICA_DIM_OBS])
{{
    float a = politica_bruta(obs);
    if (a < -1.0f) a = -1.0f;
    else if (a > 1.0f) a = 1.0f;
    return a * POLITICA_A_MAX;
}}

void politica_observar(float x_m, float v_ms, float psi_rad, float omega_rads,
                       float a_ant_ms2, float obs[POLITICA_DIM_OBS])
{{
    obs[POL_X]     =  x_m / POLITICA_X_LIM;
    obs[POL_V]     =  v_ms / POLITICA_V_MAX;
    obs[POL_SIN]   =  sinf(psi_rad);
    obs[POL_COS]   =  cosf(psi_rad);
    obs[POL_OMEGA] = -omega_rads / POLITICA_PSID_ESC;
    obs[POL_A_ANT] =  a_ant_ms2 / POLITICA_A_MAX;
}}
"""
    with open(destino, 'w') as f:
        f.write(txt)
    return err_lut


def escrever_oraculo(obs, ref32, ref64, destino, origem):
    n = len(obs)
    txt = CABECALHO.format(origem=origem, dim_obs=DIM_OBS, oculta=OCULTA,
                           dim_acao=DIM_ACAO, n_par=0, n_bytes=0)
    txt += f"""
#ifndef ORACULO_H
#define ORACULO_H

#define ORACULO_N  {n}

/* {n} vetores de observacao e a saida do ATOR em torch float32.
 * Os primeiros {25} sao casos de borda; o resto e amostragem do envelope,
 * com sin^2+cos^2 = 1.
 */
static const float ORACULO_OBS[ORACULO_N][{DIM_OBS}] = {{
{matriz_c(obs)}
}};

/* Referencia float32 (torch) */
static const float ORACULO_REF[ORACULO_N] = {{
{vetor_c(ref32)}
}};

/* Referencia float64 dos MESMOS pesos, truncada a float32. Serve para
 * separar erro de implementacao (grande) de erro de precisao (~1e-7).
 */
static const float ORACULO_REF64[ORACULO_N] = {{
{vetor_c(ref64.astype(np.float32))}
}};

#endif /* ORACULO_H */
"""
    with open(destino, 'w') as f:
        f.write(txt)


TESTE_C = r"""/* GERADO POR exportar_c.py -- NAO EDITAR A MAO.
 *
 * Roda o oraculo contra politica.c e reporta o erro maximo.
 *
 *   cc -O2 -Wall -Wextra -o teste_politica teste_politica.c politica.c -lm
 *   ./teste_politica
 *
 * Com a tabela de tanh:
 *   cc -O2 -DPOLITICA_TANH_LUT -o teste_lut teste_politica.c politica.c -lm
 *
 * Critério: erro contra o torch float32 abaixo de 1e-5 é arredondamento.
 * Acima de 1e-3 é bug de indexação ou transposição, não precisão.
 */
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "politica.h"
#include "oraculo.h"

int main(void)
{
    double pior32 = 0.0, pior64 = 0.0, soma = 0.0, pior_a = 0.0;
    int i, i_pior = 0, i_pior_a = 0, n_sat = 0;

    politica_iniciar();

    for (i = 0; i < ORACULO_N; i++) {
        float y = politica_bruta(ORACULO_OBS[i]);
        double e32 = fabs((double)y - (double)ORACULO_REF[i]);
        double e64 = fabs((double)y - (double)ORACULO_REF64[i]);
        if (e32 > pior32) { pior32 = e32; i_pior = i; }
        if (e64 > pior64) pior64 = e64;
        soma += e32;

        /* O erro que importa e o da ACELERACAO, depois do clip. Onde a rede
         * satura (|saida| >= 1) o clip engole o erro inteiro, e a maioria dos
         * piores vetores crus esta justamente ali. */
        double r = ORACULO_REF[i];
        if (r < -1.0) r = -1.0; else if (r > 1.0) r = 1.0;
        double ea = fabs((double)politica_aceleracao(ORACULO_OBS[i])
                         - r * (double)POLITICA_A_MAX);
        if (ea > pior_a) { pior_a = ea; i_pior_a = i; }
        if (fabs((double)ORACULO_REF[i]) >= 1.0) n_sat++;
    }

    printf("vetores            %d  (%d saturados, clip em +-1)\n",
           ORACULO_N, n_sat);
    printf("erro vs torch f32  max %.3e  medio %.3e\n",
           pior32, soma / ORACULO_N);
    printf("erro vs ref  f64   max %.3e\n", pior64);
    printf("erro na aceleracao max %.3e m/s2  (de %.1f) no vetor #%d\n",
           pior_a, (double)POLITICA_A_MAX, i_pior_a);
    printf("pior vetor         #%d  obs = [", i_pior);
    for (i = 0; i < POLITICA_DIM_OBS; i++)
        printf("%s%+.4f", i ? ", " : "", ORACULO_OBS[i_pior][i]);
    printf("]\n");
    printf("                   C %+.7f  torch %+.7f\n",
           politica_bruta(ORACULO_OBS[i_pior]), ORACULO_REF[i_pior]);

    /* latencia no PC: nao diz nada sobre o S3, mas pega regressao grosseira */
    {
        struct timespec t0, t1;
        volatile float acc = 0.0f;
        const int N = 20000;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        for (i = 0; i < N; i++) acc += politica_bruta(ORACULO_OBS[i % ORACULO_N]);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double us = ((t1.tv_sec - t0.tv_sec) * 1e9 +
                     (t1.tv_nsec - t0.tv_nsec)) / 1e3 / N;
        printf("latencia (PC)      %.3f us/inferencia\n", us);
    }

    if (pior32 > 1e-3) {
        printf("\nFALHOU: erro grande demais para ser precisao. Procure\n"
               "        transposicao de W ou troca de entradas na observacao.\n");
        return 1;
    }
    if (pior32 > 1e-5)
        printf("\nATENCAO: acima de 1e-5. Esperado se POLITICA_TANH_LUT estiver\n"
               "         definido; suspeito se nao estiver.\n");
    else
        printf("\nOK: dentro do arredondamento de float32.\n");
    return 0;
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('modelo', nargs='?', default='modelos_sw/melhor.pt')
    ap.add_argument('--saida', default='firmware/politica')
    args = ap.parse_args()

    if not os.path.exists(args.modelo):
        raise SystemExit(f'nao achei {args.modelo}')
    os.makedirs(args.saida, exist_ok=True)

    cam = ler_pesos(args.modelo)
    rede = carregar_torch(args.modelo)
    origem = os.path.basename(args.modelo)

    n_par = escrever_h(cam, os.path.join(args.saida, 'politica.h'), origem)
    err_lut = escrever_c(cam, os.path.join(args.saida, 'politica.c'), origem)

    obs, ref32, ref64 = gerar_oraculo(rede)
    escrever_oraculo(obs, ref32, ref64,
                     os.path.join(args.saida, 'oraculo.h'), origem)
    with open(os.path.join(args.saida, 'teste_politica.c'), 'w') as f:
        f.write(TESTE_C)

    macs = sum(W.size for W, _ in cam)
    print(f'Origem      {args.modelo}')
    print(f'Parametros  {n_par}  ({4*n_par} bytes em float32 = '
          f'{4*n_par/1024:.1f} KiB)')
    print(f'Descartado  critico + log_std ({n_par + 1} parametros)')
    print(f'Custo       {macs} MACs + {2*OCULTA} tanh por inferencia')
    print(f'Oraculo     {len(obs)} vetores | ref f32 e f64')
    print(f'LUT tanh    {LUT_N} intervalos, erro teorico {err_lut:.2e}, '
          f'{4*(LUT_N+1)} bytes')
    print(f'\nEscrito em {args.saida}/:')
    for nome in ('politica.h', 'politica.c', 'oraculo.h', 'teste_politica.c'):
        p = os.path.join(args.saida, nome)
        print(f'  {nome:18s} {os.path.getsize(p):>7d} bytes')
    print(f'\nValidar:\n'
          f'  cd {args.saida} && cc -O2 -Wall -Wextra -o teste_politica '
          f'teste_politica.c politica.c -lm && ./teste_politica')


if __name__ == '__main__':
    main()
