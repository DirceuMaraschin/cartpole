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

- [x] **APROVADO em 26/10:** swing-up captura em 3,6 s na primeira tentativa.

Se não equilibrar, pare aqui. Algo na recompilação quebrou, e não adianta
prosseguir.

---

## Etapa 1 — Confirmar que a velocidade está em 100%

O firmware **inicia em 100%** (0,45 m/s), e deve ficar assim.

```
k   →  mostra todas as constantes, inclusive o fator do modo RL
v   →  alterna 100% → 50% → 75%  (NÃO use para os testes)
```

- [x] **APROVADO em 26/10:** fator em 100%, `vmax=0.450` na telemetria.

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

## Etapa 2 — Verificar a consistência sem hardware

```bash
python teste_ponte.py
```

Compara a observação que a ponte monta com a que o simulador entrega.

- [x] **APROVADO:** `teste_ponte.py` passa; erro dentro de meio passo de quantização.

Se reprovar, as constantes de `rl_bridge.py` não batem com `pendulo_env.py`.
Nenhum treinamento conserta isso.

---

## Etapa 3 — Observação: o agente decide, o PID atua

Este é o teste mais importante antes de qualquer movimento comandado pelo agente.
O firmware entra no modo RL, publica o estado a 50 Hz e recebe a ação da rede —
mas **quem segura a haste continua sendo o PID**.

Por que não usar a ação em zero: o pêndulo cairia em ~0,6 s e sobrariam 30
amostras, pouco para conferir o que importa. Com o PID segurando, o teste dura
o que você quiser, e dá para comparar as duas decisões no mesmo estado.

```bash
source .venv/bin/activate
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
       --log logs/observa_01.csv --auto
```

Com `--auto` a ponte executa a sequência sozinha, esperando a resposta do ESP32
a cada passo. A única coisa que ela pede é que você confirme, com Enter, que a
haste está pendurada e parada — o resto é automático.

> **Por que automatizar.** Digitar comandos enquanto o firmware fala pela mesma
> porta é a receita para errar o tempo: a telemetria rola na tela e você não vê
> mais o que digitou. A telemetria agora fica escondida por padrão (`--verboso`
> mostra), e `--auto` tira a digitação da equação.

> **O Monitor Serial fica FECHADO durante todo o teste.** No macOS a porta é
> exclusiva: ou a IDE a usa, ou a ponte. Você digita os comandos no mesmo
> terminal onde a ponte está rodando, seguidos de Enter, e ela repassa ao
> ESP32. As respostas do firmware aparecem prefixadas com `[esp32]`.

**Não toque no `o` manualmente.** A ponte cuida dele: o `o` do firmware é um
*toggle*, então ela lê a resposta e insiste até o firmware confirmar
`LIGADO`. Depois do `r` ela ainda confere a flag `F_OBSERVA` no pacote e
**aborta** se o estado não for o pedido. Mexer no `o` por fora só atrapalha
essa conferência.

Você só faz duas coisas: teclar Enter quando ela pedir (haste pendurada e
parada) e `q` ao terminar.

Antes de ir para a bancada, rode a rede de proteção, que não precisa de
hardware nenhum:

```bash
python teste_auto.py
```

Ela sobe um ESP32 falso — com o watchdog de 60 ms e o toggle do `o` — e exige
que, nos 4 cenários, a observação termine no estado pedido e a taxa fique
perto de 50 Hz.

A cada segundo aparece uma linha assim:

```
  >> modo RL: OBSERVACAO, quem atua e o PID
  312 passos | psi  0.31deg | x   +7.2mm | a  +0.42 | perdidos 0 | rede 0.12ms
      | OBSERVA: PID +0.38 agente +0.42, mesmo sinal  94%
```

- [ ] **Aprovação:**
  - a linha `>> modo RL:` diz **OBSERVACAO, quem atua e o PID**
  - taxa próxima de **50 Hz** (o contador sobe ~50 por segundo)
  - **perdidos = 0** ou pouquíssimos
  - **latência da rede < 1 ms** (a rede é aquecida na partida)
  - **mesmo sinal acima de 60%** e correlação acima de 0,3 no resumo final

> **Se parar em poucos passos.** Foi o watchdog de 60 ms: alguma coisa no PC
> travou o laço e a ação não chegou. Procure a flag `16` (`F_PERDEU`) somada
> na coluna `flags` do CSV — `52` = 32+16+4 significa "estou em RL, zerado, e
> perdi a ação anterior". Foi exatamente o que aconteceu em 29/10.

> **O que esse teste pega.** Se a convenção de ângulo, a direção do motor ou o
> `ENC_SINAL` estiverem invertidos em algum ponto da cadeia, o agente decidirá
> sistematicamente no sentido contrário ao do PID — e o resumo final acusa.
> Sinal invertido é o erro mais destrutivo possível: o agente empurraria o
> carrinho para o lado da queda. Descobrir aqui custa nada; descobrir na etapa
> seguinte custa a haste.

Deixe rodar uns 30 segundos e encerre com `q`.

## Etapa 4 — Primeiro episódio real

Mão na chave de emergência. Alguém pronto para segurar a haste.

```bash
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
       --segundos 30 --log logs/rl_01.csv --auto --sem-observacao
```

O `--sem-observacao` é o que muda a etapa 3 para a etapa 4: é ele que tira o
PID do comando. A ponte avisa na partida, em letras maiúsculas, que o agente
vai mover o carrinho de verdade — e, como na etapa 3, ela confere a flag
`F_OBSERVA` depois do `r` e aborta se o estado não for o pedido.

> Confira a linha `>> modo RL:`. Ela deve dizer **o AGENTE esta no comando**.
> Se disser OBSERVACAO, o PID continua no comando e você vai achar que o
> agente está funcionando quando não está.

O firmware só entrega ao agente quando a haste estiver **estável**:
|ψ| < 3°, |ψ̇| < 0,5 rad/s e |x| < 120 mm. Entregar um pêndulo já caindo seria
preparar o agente para falhar.

- [ ] **Aprovação:** o agente segura a haste por pelo menos 10 s sem o supervisor
      intervir.

**O que pode acontecer e o que significa:**

| Sintoma | Causa provável | O que fazer |
|---|---|---|
| Supervisor assume na hora | sinal invertido, ou zero mal calibrado | volte à etapa 3; o resumo de observação teria acusado |
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

## O que a bancada já entregou (26/10/2026)

| Grandeza | Valor medido | Como ficou |
|---|---|---|
| Swing-up até capturar | 3,55 s | simulador previa 3,60 s |
| Capturas perdidas | 0 | — |
| \|psi\| em regime | 0,30° | meta era < 2° |
| \|x\| em regime | 7 mm | meta era < 120 mm |
| Freio de parede no swing-up | 0 de 71 amostras | trilho não é gargalo |

Ajustes que produziram isso, todos já no firmware:

- `ALVO_EN = 1,10` — o alvo de energia era 1,00, a trajetória marginal que
  nunca transpõe o topo. Antes: 28% do swing-up preso entre En 0,90 e 1,00.
- `KX = 5,0` — era 1,5, e o carrinho parava a 73 mm do centro.
- `L_EQ_M = 0,220` — é sintonia do controlador de energia, não medida física.
- Limitador de velocidade restrito ao modo RL (travava o swing-up).

## Escala do trilho: resolvida em 26/10

Os 40 000 passos/m estão corretos, confirmados por três caminhos independentes
que concordam dentro de 0,5% (ver `identificacao.md`, seção 6).

A divergência anterior era de medida, não de hardware: a régua dava o **vão
livre** entre as chaves (600 mm), enquanto o firmware mede o **percurso do
carrinho**, que é o vão menos a largura do carrinho (65,5 mm) — 534,5 mm.

`CURSO_REAL_M` fica em **0** de propósito: assim o firmware usa o valor teórico,
que vem de aritmética exata (20 dentes × 2 mm ÷ 1600 pulsos) e é mais preciso
que uma medida de régua com ±2 mm de incerteza.
