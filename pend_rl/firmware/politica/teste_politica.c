/* GERADO POR exportar_c.py -- NAO EDITAR A MAO.
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
