/* GERADO POR exportar_c.py -- NAO EDITAR A MAO.
 *
 * Origem : melhor.pt
 * Modelo : ator 6 -> 64 -> 64 -> 1, tanh, cabeca linear
 * Pesos  : 4673 parametros, 18692 bytes em float32
 */

#ifndef POLITICA_H
#define POLITICA_H

#ifdef __cplusplus
extern "C" {
#endif

#define POLITICA_DIM_OBS   6
#define POLITICA_OCULTA    64
#define POLITICA_DIM_ACAO  1
#define POLITICA_N_PARAM   4673

/* Constantes de normalizacao -- geradas de pendulo_env.py e pendulo_env_sw.py.
 *
 * NAO as redigite em outro lugar do firmware. Se uma escala divergir entre o
 * PC e a placa, o agente passa a ver um mundo que nao e o dele e o sintoma
 * aparece so na bancada, como "o agente ficou pior".
 */
#define POLITICA_X_LIM       0.24f      /* m     limite de software do trilho */
#define POLITICA_V_MAX       0.45f      /* m/s   escala da velocidade         */
#define POLITICA_PSID_ESC    15.0f     /* rad/s escala da velocidade angular */
#define POLITICA_A_MAX       8.0f      /* m/s2  saturacao da acao            */
#define POLITICA_DT          0.02f     /* s     passo do agente (50 Hz)      */

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
enum {
    POL_X = 0, POL_V = 1, POL_SIN = 2, POL_COS = 3, POL_OMEGA = 4, POL_A_ANT = 5
};

/* Copia os pesos de flash para SRAM interna. Chamar uma vez no setup().
 *
 * Por que copiar: const vive em .rodata, atras do cache de flash. Um miss no
 * meio da inferencia e jitter, e jitter num sistema que dobra o erro a cada
 * 105 ms nao e detalhe. Custa 18692 bytes dos 512 KiB de SRAM.
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
}
#endif

#endif /* POLITICA_H */
