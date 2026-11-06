# Pêndulo invertido com PPO embarcado

Pêndulo invertido físico (carro sobre trilho, haste livre) controlado por um
agente PPO que roda **dentro do ESP32**. Não há PC no laço de controle e não há
controle clássico no caminho do agente.

## Resultado (bancada, 180 s, agente completo)

| | |
|---|---|
| swing-up | 2,47 s |
| em pé (\|ψ\|<10°) | 99,5% |
| ⟨\|ψ\|⟩ | 0,75° |
| quedas | nenhuma em 178 s |
| inferência | 335 µs de p99 — 2,1% do passo de 20 ms |
| rede | 6→64→64→1, 4 673 parâmetros, 18,7 kB |

## Estrutura

    logs/                   dados brutos da identificação do sistema (out/15)
                            livre_01, pid_01, swing_01/02 -- deles saíram L_eq
                            e os coeficientes de atrito
    pendulo_log.ino         firmware de registro usado nessa identificação
    gravar_log.py           lado PC do registro
    pid.py                  simulação inicial do PID
    teste_comunicacao.py    teste de comunicação inicial
    encoder_esp32s3.ino     teste de encoder para o ESP32-S3
    testependulo/           firmware clássico original (PID + swing-up por energia)

    pend_rl/                o projeto de RL
      pendulo_env.py          simulador (física identificada dos logs)
      pendulo_env_sw.py       variante com swing-up: obs de 6 entradas, sin/cos
      pendulo_env_v3.py       variante com recompensa v3
      ppo.py                  PPO: ator-crítico, GAE, clip, KL early stop
      treinar.py / treinar_sw.py     treino (equilíbrio / completo)
      avaliar.py / avaliar_sw.py     avaliação em simulação
      comparar.py                    comparação entre modelos

      exportar_c.py           .pt -> politica.c/.h, oraculo.h, teste_politica.c
      politica_c.py           carrega politica.c via ctypes (o mesmo .c da placa)
      comparar_c.py           malha fechada: torch x C, contra piso de ruído medido
      teste_cadeia.py         regressão da cadeia de a_ant

      rl_bridge.py            ponte serial (hoje só para registro, fora do laço)
      conferir_local.py       conferência bit a bit placa x PC (modo observador)
      registrar_local.py      etapas 2 e 3 com o agente embarcado atuando
      teste_ponte*.py, teste_completo.py, teste_travada.py, teste_auto.py
                              regressões da era em que a ponte estava no laço

      modelos/                agente v1 (só equilíbrio) -- passou as 9 etapas
      modelos_sw/             agente completo -- melhor.pt é o EMBARCADO
      _arquivo/               repetições de corrida e o ensaio curto de treino

      analise/                scripts da identificação dos parâmetros físicos.
                              Usam caminhos relativos de uma estrutura antiga;
                              valem como registro da derivação, não para rodar.
      logs/                   corridas de bancada da era da ponte
      logs/embarcado/         as 4 corridas que são o registro da validação
                              embarcada: conf_02 (bit a bit), et2_01 (etapa 2),
                              et3_02 (180 s), et3_solto (300 s, autonomia)
      docs/ -- na raiz de pend_rl: roteiro_bancada.md, identificacao.md,
               ppo_explicado.md

    firmware/
      pendulo_rl/           O QUE ESTA GRAVADO NA PLACA
                            pendulo_rl.ino + politica.c/.h + oraculo.h + estado_rl.h
      politica/             saída canônica do exportar_c.py + teste em C
      bench_politica/       benchmark de latência isolado
      classico/pendulo_pid/ só o controle clássico, sem RL

As três cópias de `politica.c`, `politica.h` e `oraculo.h` são idênticas por
construção (o Arduino só compila o que está na pasta do sketch). Ao trocar de
modelo, rode `exportar_c.py` e copie para as três.

## Como rodar

    # ambiente
    source venv/bin/activate && cd pend_rl

    # validar o C antes de gravar
    cd firmware/politica && cc -O2 -o teste_politica teste_politica.c politica.c -lm && ./teste_politica
    cd ../.. && python politica_c.py && python comparar_c.py && python teste_cadeia.py

    # bancada (porta: arduino-cli board list)
    python conferir_local.py --porta /dev/cu.usbserial-0001 --seg 30 --csv conf.csv
    python registrar_local.py --porta ... --etapa 2 --seg 60 --csv et2.csv
    python registrar_local.py --porta ... --etapa 3 --seg 180 --csv et3.csv

    # só o controle clássico
    # grave firmware/classico/pendulo_pid/

Teclas do firmware, pelo monitor serial a 921600:
`z` zera o encoder (haste pendurada e parada) · `c` centraliza ·
`l` rede local (roda na placa) · `o` observador (PID atua) ·
`g` agente completo · `r` entrega após captura · `s` para ·
`m` supervisor PID · `v` limitador de velocidade

Depois de um reset (EN) os três alternadores voltam a desligado: para voltar a
equilibrar, `z` `c` `l` `g`.
