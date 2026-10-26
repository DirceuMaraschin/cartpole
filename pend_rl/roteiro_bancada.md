# Roteiro de bancada: do simulador ao pêndulo real

Siga **na ordem**. Cada etapa tem um critério de aprovação; não avance sem ele.
As três primeiras não movem o motor em modo RL — são de segurança.

---

## Antes de tudo: as três camadas de proteção

O agente é a parte menos confiável do sistema. Tudo foi montado assumindo que ele
vai errar.

| Camada | Onde | O que faz |
|---|---|---|
| 1. Supervisor PID | firmware | \|ψ\| > 20° → o PID retoma o controle (ele tem até 35° para salvar) |
| 2. Watchdog | firmware | sem pacote por 60 ms → freia ou devolve ao PID |
| 3. Saturações e freio de parede | firmware | velocidade, aceleração e limite de curso. O agente **não** escapa disso |
| 4. Abortar pelo PC | `rl_bridge.py` | \|ψ\| > 18° → manda `CMD_PID` |
| 5. Fins de curso | hardware (ISR) | corta os pulsos direto na interrupção |
| 6. Chave de emergência | você | em série com os 24 V, ao alcance da mão |

> A camada 6 não é opcional. Nas primeiras horas, alguém fica com a mão nela.

---

## Etapa 0 — Compilar e conferir o que não mudou

```
firmware/pendulo_rl.ino  →  Arduino IDE  →  ESP32 Dev Module  →  921600 baud
```

**Confira primeiro que o PID continua igual.** Envie `z`, depois `i`.
O swing-up e o equilíbrio devem funcionar exatamente como antes.

- [ ] **Aprovação:** o PID equilibra como sempre equilibrou.

Se não equilibrar, pare aqui. Algo na recompilação quebrou, e não adianta
prosseguir.

---

## Etapa 1 — Confirmar que a velocidade está em 100%

O firmware **inicia em 100%** (0,45 m/s), e deve ficar assim.

```
k   →  mostra todas as constantes, inclusive o fator do modo RL
v   →  alterna 100% → 50% → 75%  (NÃO use para os testes)
```

- [ ] **Aprovação:** o comando `k` mostra `v_max_rl_ms` igual a `v_max_ms`, com fator 100%.

> **Por que não começar devagar.** Parece prudente, mas não é: o agente treinou
> com v_max de 0,45 m/s. A 75% ele cai de 92% para 64% de sucesso; a 50%, para
> **zero**. Reduzir a velocidade não é começar com cuidado — é colocar o agente
> num mundo que ele nunca viu, e você depuraria o problema errado.
>
> E o swing-up precisa de velocidade plena para bombear energia: abaixo de ~65%
> a haste não passa da horizontal. Por isso o limitador não toca nele.
>
> A proteção do primeiro episódio vem do **supervisor PID** e da **chave de
> emergência**. Só reduza o fator se treinar um agente para o v_max reduzido.

---

## Etapa 2 — Observação: o agente decide, o PID atua

Este é o teste mais importante antes de qualquer movimento comandado pelo agente.
O firmware entra no modo RL, publica o estado a 50 Hz e recebe a ação da rede —
mas **quem segura a haste continua sendo o PID**.

Por que não usar a ação em zero: o pêndulo cairia em ~0,6 s e sobrariam 30
amostras, pouco para conferir o que importa. Com o PID segurando, o teste dura
o que você quiser, e dá para comparar as duas decisões no mesmo estado.

```bash
source .venv/bin/activate
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
       --log logs/observa_01.csv
```

No terminal: `z` → `i` → espere capturar e centralizar → `o` → `r`.

A cada segundo aparece uma linha assim:

```
  312 passos | psi  0.31deg | x   +7.2mm | a  +0.42 | perdidos 0 | rede 0.21ms
      | OBSERVA: PID +0.38 agente +0.42, mesmo sinal  94%
```

- [ ] **Aprovação:**
  - taxa próxima de **50 Hz** (o contador sobe ~50 por segundo)
  - **perdidos = 0** ou pouquíssimos
  - **latência da rede < 2 ms**
  - **mesmo sinal acima de 60%** e correlação acima de 0,3 no resumo final

> **O que esse teste pega.** Se a convenção de ângulo, a direção do motor ou o
> `ENC_SINAL` estiverem invertidos em algum ponto da cadeia, o agente decidirá
> sistematicamente no sentido contrário ao do PID — e o resumo final acusa.
> Sinal invertido é o erro mais destrutivo possível: o agente empurraria o
> carrinho para o lado da queda. Descobrir aqui custa nada; descobrir na etapa
> seguinte custa a haste.

Deixe rodar uns 30 segundos e encerre com `q`.

## Etapa 3 — Verificar a consistência sem hardware

```bash
python teste_ponte.py
```

Compara a observação que a ponte monta com a que o simulador entrega.

- [ ] **Aprovação:** "APROVADO: toda divergência cabe dentro de meio passo".

Se reprovar, as constantes de `rl_bridge.py` não batem com `pendulo_env.py`.
Nenhum treinamento conserta isso.

---

## Etapa 4 — Primeiro episódio real

Mão na chave de emergência. Alguém pronto para segurar a haste.

```bash
python rl_bridge.py --porta /dev/cu.usbserial-0001 \
    --modelo modelos/melhor.pt --segundos 30 --log logs/rl_01.csv
```

Sequência: `z` → `e` → levante a haste → `r`.

O firmware só entrega ao agente quando a haste estiver **estável**:
|ψ| < 3°, |ψ̇| < 0,5 rad/s e |x| < 50 mm. Entregar um pêndulo já caindo seria
preparar o agente para falhar.

- [ ] **Aprovação:** o agente segura a haste por pelo menos 10 s sem o supervisor
      intervir.

**O que pode acontecer e o que significa:**

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| Supervisor assume na hora | sinal invertido, ou zero mal calibrado | volte à etapa 2 e confira `a` contra a inclinação |
| Oscila junto e cai em 2-3 s | atraso real maior que o modelado | aumente `atraso_passos` para 2 e retreine |
| Equilibra mas deriva até a parede | offset de zero do encoder | recalibre com `z`, haste bem parada |
| Trepida muito (barulho, vibração) | política nervosa | suba o coeficiente de `\|Δa\|` na recompensa e retreine |
| `perdidos` sobe sem parar | serial congestionada | feche o Monitor Serial da IDE |

---

## Etapa 5 — Supervisor desligado (só depois do sucesso na etapa 4)

```
m   →  desliga o supervisor PID
```

Agora o agente está sozinho até os 35° do firmware. Este teste mostra a
capacidade real dele de recuperar.

- [ ] **Aprovação:** aguenta 20 s sem o PID por trás.

> Se reprovar aqui mas tiver passado na etapa 4, não é fracasso: significa que
> o agente controla bem mas recupera mal. Religue o supervisor (`m`) e use
> assim — é uma arquitetura legítima.

---

## Etapa 6 — Conferir perda de passos

Já estamos a 100% desde a etapa 1, então aqui o objetivo é outro: confirmar que
o motor não perdeu passos durante os episódios.

- [ ] **Aprovação:** nenhuma deriva na posição física após 3 episódios completos.

**Como detectar.** A 100% e 8 m/s², é o momento de maior risco.
Para detectar: depois do episódio, envie `s` e `a`, e confira se o carrinho
para no mesmo lugar físico de sempre. Se estiver deslocado, perdeu passos —
reduza a aceleração ou aumente a corrente do DRV8825.

---

## Etapa 7 — Medir o desempenho real

Rode 3 episódios de 60 s gravando o log, depois compare com o simulador:

```bash
python rl_bridge.py --porta ... --segundos 60 --log logs/rl_final_01.csv
```

| Métrica | Simulador previu | Bancada real |
|---|---|---|
| sobrevive 20 s | 87% | ? |
| ⟨\|ψ\|⟩ | 1,98° | ? |
| ⟨\|x\|⟩ | 32 mm | ? |

Se a bancada ficar **muito** pior que o simulador, o modelo tem algo errado e
vale voltar à identificação com os novos logs. Se ficar próxima, a transferência
funcionou.

---

## Etapa 8 — Testes de perturbação

Com o agente equilibrando, dê **toquinhos leves** na haste com o dedo.
Comece muito suave e aumente aos poucos.

O simulador prevê 90% de recuperação para chutes de 1,0 rad/s, e **zero**
acima de ~16° de inclinação — esse é o teto físico do atuador, não do agente.

- [ ] **Aprovação:** recupera de toques leves de forma visivelmente mais suave
      que o PID.

---

## Resumo dos comandos

| Tecla | Função |
|---|---|
| `z` | define o zero (haste pendurada e parada) |
| `e` | centraliza e espera você levantar a haste |
| `i` | swing-up automático + PID |
| `r` | **entrega o controle ao agente** |
| `s` | para tudo |
| `m` | liga/desliga o supervisor PID |
| `v` | limitador do modo RL (deixe em 100%) |
| `k` | mostra as constantes |
| `p` | posição e ângulo |
| `a`/`b`/`c` | vai ao lado A / lado B / centro |

---

## Pendência que continua em aberto

`CURSO_REAL_M` ainda está em 0 (escala teórica de 40 000 passos/m). A medida
decisiva está em `identificacao.md`, seção 6: marque o carrinho nas posições
`a` e `b`, meça entre as marcas, e divida 21 316 pela distância em metros.

O agente foi treinado com ±8% de incerteza nessa escala, então ele aguenta.
Mas é informação gratuita que vale a pena ter.
