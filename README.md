# Pêndulo invertido com PPO embarcado

Pêndulo invertido físico (carro sobre trilho GT2, haste livre, encoder de
600 PPR, motor de passo com DRV8825) controlado por um agente PPO que roda
**dentro do ESP32**. Não há PC no laço de controle e não há controle clássico
no caminho do agente.

## Resultado (bancada, 180 s, agente completo)

| | |
|---|---|
| swing-up | 2,47 s |
| em pé (\|ψ\|<10°) | 99,5% |
| ⟨\|ψ\|⟩ | 0,75° |
| quedas | nenhuma em 178 s |
| \|x\| máx na subida | 134 mm de 240 |
| inferência | 335 µs de p99 — 2,1% do passo de 20 ms |
| rede | 6→64→64→1 tanh, 4 673 parâmetros, 18,7 kB |
| conferência placa × PC | 5,4e-06 em 8 959 amostras |

## Estrutura

    pend_rl/
      pendulo_env.py        simulador: física identificada da bancada
      pendulo_env_sw.py     variante com swing-up (obs de 6 entradas, sin/cos)
      ppo.py                PPO: ator-crítico, GAE, clip, KL early stop
      treinar_sw.py         treino do agente completo
      avaliar_sw.py         avaliação em simulação, 6 baterias
      identificacao.md      de onde veio cada constante física
      requirements.txt

      exportar_c.py         .pt -> politica.c/.h (sketch) + oraculo.h e
                            teste_politica.c (pasta de teste)
      politica_c.py         carrega politica.c via ctypes -- o MESMO .c da placa
      comparar_c.py         malha fechada torch × C, contra piso de ruído medido
      teste_cadeia.py       regressão da cadeia de a_ant

      rl_bridge.py          ponte serial -- hoje só leitura, fora do laço
      conferir_local.py     conferência bit a bit placa × PC (modo observador)
      registrar_local.py    etapas 2 e 3 com o agente embarcado atuando

      modelos_sw/           melhor.pt é o EMBARCADO; agente.pt é a it 2000

      firmware/
        pendulo_rl/         O QUE ESTÁ GRAVADO NA PLACA, e a ÚNICA cópia dos
                            pesos: pendulo_rl.ino, politica.c, politica.h,
                            estado_rl.h
        politica/           oraculo.h + teste_politica.c -- o teste em C.
                            Separado porque teste_politica.c tem main() e
                            colidiria com o do core dentro de um sketch.
        bench_politica/     benchmark de latência isolado. Precisa que os três
                            arquivos sejam copiados para cá antes de gravar;
                            o cabeçalho do .ino diz como.
        classico/pendulo_pid/  só o controle clássico, sem RL

**Uma cópia só dos pesos.** `politica.c` existe em um lugar: a pasta gravada.
O teste e o .so compilam contra ela. Isso elimina a divergência que já
aconteceu uma vez entre três cópias.

## Como rodar

    source venv/bin/activate && cd pend_rl

    # 1. gerar o C de um modelo treinado
    python exportar_c.py modelos_sw/melhor.pt

    # 2. o C reproduz o torch?            criterio: < 1e-5
    cd firmware/politica
    cc -O2 -Wall -Wextra -I../pendulo_rl -o teste_politica \
       teste_politica.c ../pendulo_rl/politica.c -lm && ./teste_politica
    cd ../..

    # 3. fidelidade e malha fechada        criterio: variantes em C "ok"
    python politica_c.py && python comparar_c.py && python teste_cadeia.py

    # 4. latencia na placa                 criterio: oraculo < 1e-5, p99 < 1000us
    #    copie politica.c/.h e oraculo.h para firmware/bench_politica/ e grave

    # 5. grave firmware/pendulo_rl/ (ESP32 Dev Module, 240 MHz, 921600)

    # 6. bancada, em ordem crescente de risco (porta: arduino-cli board list)
    python conferir_local.py  --porta /dev/cu.usbserial-0001 --seg 30  --csv conf.csv
    python registrar_local.py --porta ... --etapa 2 --seg 60  --csv et2.csv
    python registrar_local.py --porta ... --etapa 3 --seg 180 --csv et3.csv

## Teclas do firmware (monitor serial a 921600)

`z` zera o encoder (haste pendurada e parada) · `c` centraliza ·
`l` rede local, roda na placa · `o` observador, o PID atua ·
`g` agente completo · `r` entrega após captura · `s` para ·
`m` supervisor PID · `v` limitador de velocidade

Depois de um reset (EN) os três alternadores voltam a desligado: para voltar a
equilibrar, `z` `c` `l` `g`. O PC não é necessário no laço — só para digitar
essas teclas e registrar.

## O que está só no histórico

O diretório de trabalho guarda o necessário para operar e evoluir. As etapas
anteriores ficaram nos commits:

| o que | onde |
|---|---|
| logs brutos da identificação (`logs/`) | `git show a3984c8:logs/livre_01.csv` |
| scripts da identificação (`pend_rl/analise/`) | `git show 1bfbf23:pend_rl/analise/` |
| agente v1, só equilíbrio (`modelos/`) | `git show 1bfbf23:pend_rl/modelos/melhor.pt` |
| corridas de bancada da era da ponte | `git show a3984c8:pend_rl/logs/` |
| registro da validação embarcada | `git show a3984c8:pend_rl/logs/embarcado/` |
| testes da ponte-no-laço | `git show 1bfbf23:pend_rl/teste_auto.py` |
| firmware clássico original | `git show 1bfbf23:testependulo/testependulo.ino` |

As conclusões da identificação — valor de cada constante e como foi obtida —
estão em `pend_rl/identificacao.md`, que por isso ficou.
