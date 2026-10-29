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

- [x] **APROVADO em 29/10** (`logs/observa_02.csv`, 76 s, 3780 passos):

  | Critério | Meta | Medido |
  |---|---|---|
  | taxa | ~50 Hz | **49,7 Hz** |
  | perdidos | 0 | **0** |
  | latência da rede | < 1 ms | **0,61 ms** (pior 7,68) |
  | mesmo sinal | > 60% | **80%** |
  | correlação | > 0,3 | **+0,916** |
  | ⟨\|ψ\|⟩ | — | **0,19°** (máx 0,60°) |

  A correlação de +0,916 é o resultado que libera a etapa seguinte: não há
  sinal invertido em `ENC_SINAL`, em ψ nem na direção do motor.

  **Por que o agente pede mais que o PID** (+0,83 contra +0,10 m/s² de média):
  o carrinho ficou parado em +66 mm, e o agente é ~6× mais agressivo em
  centralizar que o PID com `KX=5,0`. Medido no simulador, no estado real da
  bancada: a 66 mm ele pede +1,95 m/s²; no centro, −0,02. O offset do encoder
  (−0,19°) responde por apenas −0,25 m/s² disso. Não é defeito — é a
  recompensa de posição fazendo o seu trabalho. É também o que explica
  correlação +0,92 com "mesmo sinal" de só 80%: o PID oscila em torno de zero
  enquanto o agente fica firmemente positivo, então o *sinal* discorda sem que
  as *decisões* discordem.

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

### Os três bugs que a etapa 3 revelou (29/10)

Valem registro porque dois eram antigos e invisíveis.

1. **`F_OBSERVA` nunca saía do ESP32.** No firmware, `flags |= F_OBSERVA` vinha
   *depois* de `enviarEstado(flags)`: o bit era aceso num `flags` local já
   transmitido e zerado no ciclo seguinte. Estava assim desde sempre, e nunca
   apareceu porque nada conferia a flag. Corrigido movendo a linha para antes
   do envio.
2. **A sequência automática não alimentava o watchdog.** O laço pós-`r` lia
   pacotes para confirmar `F_RL` sem nunca responder, e o firmware voltava ao
   PID em 60 ms. Sintoma: 2 passos em 64,9 s.
3. **Pacote velho respondido = comando digitado.** Fora do modo binário o
   firmware lê *cada byte* como tecla. Três pacotes atrasados viraram um `Z` e
   18 "comando desconhecido". Deu em mensagens inofensivas por sorte — um byte
   igual a `a`, `b` ou `i` manda o carrinho andar. A ponte agora checa o
   relógio antes de enviar e se cala se passou dos 60 ms.

Dois testes sem hardware guardam isso: `teste_auto.py` (os 4 cenários do
toggle) e `teste_travada.py` (injeta um travamento de 100 ms e exige zero
bytes-virando-tecla).

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

- [x] **APROVADO em 29/10, primeira tentativa** (`logs/rl_01.csv`, 1492 passos
      a 49,7 Hz): **30 s sem uma única intervenção do supervisor.**

  | Métrica | Simulador previu | Bancada |
  |---|---|---|
  | sobrevive 20 s | 87% | **30 s, 1ª tentativa** |
  | ⟨\|ψ\|⟩ | 1,98° | **1,75°** (máx 7,95°) |
  | ⟨\|x\|⟩ | 32 mm | **27 mm** |
  | perdidos | 0 | **0** |
  | ciclos acima do watchdog | 0 | **0** |

  As duas métricas saíram **melhores** que a previsão. A identificação do
  modelo (L_eq, atrito viscoso e de Coulomb) foi o que pagou essa
  transferência.

  Ação equilibrada: 45% positiva, média −0,26 m/s², pico 6,54 de 8,0 — usa o
  atuador sem saturar. Nada de empurrão sistemático.

  **Um intervalo de 47 ms** (contra os 60 ms do watchdog). A instrumentação
  disse onde: `espera da serial 45.4 ms`, com `perdidos = 0`. Ou seja, o ESP32
  publicou em sequência e o **USB entregou os pacotes em lote** — é o
  temporizador de latência do driver serial do macOS, não o coletor de lixo do
  Python nem o custo de log/impressão. Aconteceu 1 vez em 1492 ciclos.
  Monitore: se passar de ~55 ms com frequência, o episódio começa a cair por
  isso.

**Previsão, medida no simulador com o estado real da bancada:** o agente puxa o
carrinho dos ~66 mm de volta ao centro com cerca de +2 m/s² e para em torno de
**7 mm**. Com o agente no comando, o offset do encoder deixa de custar 66 mm de
curso e passa a custar 7. Se em vez disso ele **derivar até a parede**, aí sim
o offset é o culpado: refaça o `z` com a haste bem parada.

Confira também o resumo final: `ciclos acima do watchdog de 60 ms` deve ser
**0**. Se não for, as linhas `!! intervalo` dizem se o tempo foi em
log+impressão, na espera da serial ou na rede.

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

```bash
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
       --segundos 20 --log logs/rl_sem_superv_01.csv \
       --auto --sem-observacao --sem-supervisor
```

> **Não digite `m` durante o episódio — não funciona.** Em modo RL o firmware
> não lê texto: `loop()` desvia para `processarSerialRL()`, que descarta todo
> byte que não seja o cabeçalho `0x5A 0xA5`. O `m` é engolido em silêncio. E o
> protocolo binário só tem quatro comandos (nada, parar, PID, RL) — nenhum
> deles mexe no supervisor. Por isso ele é desligado **antes** do `r`, e é o
> que `--sem-supervisor` faz, conferindo a resposta do firmware.
>
> A ponte agora avisa quando uma tecla não chega: durante o modo RL só valem
> `p` (devolve ao PID pelo protocolo), `s` (para) e `q` (encerra).

Agora o agente está sozinho até os 35° do firmware. Este teste mostra a
capacidade real dele de recuperar.

> **Atenção, a rede de proteção some em dois sentidos.** Sem supervisor, se a
> comunicação cair o firmware **para o carrinho** em vez de devolver ao PID
> (`modo = PARADO`), e a haste cai. Dado o intervalo de 47 ms visto na etapa 4,
> isso é um risco concreto. Mão na chave e alguém segurando a haste.

- [x] **APROVADO em 29/10** (`logs/rl_sem_superv_01.csv`, 995 amostras, 20 s a
      49,8 Hz): aguentou os 20 s inteiros sozinho.

  `flags = 36` (`RL+ZERO_OK`) em **100%** das amostras: sem `F_OBSERVA` (o
  agente atuava de fato), sem `F_SUPERV`, e **nenhum `F_PERDEU`** — zero ações
  perdidas.

  | Métrica | Etapa 4 | Etapa 5 |
  |---|---|---|
  | ⟨\|ψ\|⟩ | 1,75° | **1,53°** (rms 1,99°, máx 6,15°) |
  | ⟨\|x\|⟩ | 27 mm | **20 mm** (média +16, faixa −45 a +77) |
  | pior intervalo | 47 ms | **22,3 ms** |

  Não é "melhor sem supervisor": na etapa 4 ele nunca disparou, logo não
  influenciava nada. A diferença é variação entre execuções — e um `z` melhor
  (viés de ψ de −0,215°). O que esta etapa prova é que o agente **não precisa**
  da rede de proteção.

  Outros números: deriva de +0,76 mm/s (indistinguível do ciclo limite em
  20 s); velocidade máxima 0,435 de 0,450 m/s — **folga apertada**, o que
  confirma por que o agente desaba a 75% e a 50%; ação tocou 8,0 m/s² em 1
  amostra; ciclo limite a 1,40 Hz (período 0,71 s), ~4,7× o tempo natural de
  queda τ = 0,151 s.

### Achado: a política tremula

⟨\|Δa\|⟩ = **2,31 m/s² por passo**, contra ⟨\|a\|⟩ = 2,00. A ação muda mais que
a própria magnitude média a cada 20 ms. O coeficiente de `|Δa|` na recompensa
(0,05) ficou fraco demais.

Não impede o equilíbrio — as duas etapas passaram acima da previsão. Mas tem
consequência mecânica: reversões rápidas de aceleração são a causa clássica de
**perda de passos** num motor de passo. Por isso a etapa 6 vem agora.

Se a etapa 6 acusar perda de passos, o conserto é retreinar com o coeficiente
de `|Δa|` maior (0,15 a 0,20) em `pendulo_env.py`, aceitando um pouco mais de
⟨\|ψ\|⟩ em troca de um comando mais suave.

> Se reprovar aqui mas tiver passado na etapa 4, não é fracasso: significa que
> o agente controla bem mas recupera mal. Religue o supervisor (`m`) e use
> assim — é uma arquitetura legítima.

---

## Etapa 6 — Conferir perda de passos

Já estamos a 100% desde a etapa 1, então aqui o objetivo é outro: confirmar que
o motor não perdeu passos durante os episódios.

- [ ] **Aprovação:** nenhuma deriva na posição física após 3 episódios completos.

**Por que agora.** Medimos ⟨\|Δa\|⟩ = 2,31 m/s² por passo na etapa 5 (ver o
achado lá): a aceleração reverte de sinal constantemente a 50 Hz. É a condição
clássica para um motor de passo perder sincronismo — mais exigente que uma
aceleração alta e constante.

**Como medir, de forma repetível.** O ponto é comparar a *crença* do firmware
sobre a posição com a realidade física.

1. Antes de tudo, mande `c` (vai ao centro) e marque com lápis ou fita onde
   o carrinho parou. Essa é a referência.
2. Rode 3 episódios completos:

   ```bash
   python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
          --segundos 30 --log logs/rl_passos_0N.csv --auto --sem-observacao
   ```

3. Depois de cada um, mande `s` e depois `c`, e confira se o carrinho volta
   **exatamente** à marca.

> **Onde digitar o `c`.** Quando a ponte encerra, ela libera a porta — então
> abra o Monitor Serial da IDE e mande `c` por lá. Durante o episódio não
> funcionaria: em modo RL o firmware não lê texto. E **não regrave o firmware**
> entre os episódios: isso refaz o homing e apaga justamente a evidência que
> você está tentando medir.

Se parar deslocado, perdeu passos. A quantidade importa: 1 mm = 40 passos.
Conserto, em ordem de preferência: aumentar a corrente do DRV8825; reduzir
`A_BAL_MAX` de 8 para 6; retreinar com o coeficiente de `|Δa|` maior.

> Um `z` **não** serve aqui: ele zera o encoder, não a posição do carrinho.
> Quem referencia o carrinho é o homing nas chaves de fim de curso.

---

## Etapa 7 — Medir o desempenho real

Rode 3 episódios de 60 s gravando o log, depois compare com o simulador:

```bash
python rl_bridge.py --porta ... --segundos 60 --log logs/rl_final_01.csv
```

- [x] **APROVADO em 29/10** — `logs/rl_final_{01,02,03}.csv`, 3 × 60 s,
      2984 amostras cada a 49,7 Hz, **os três completos**.

| Métrica | Simulador previu | Bancada (8.952 amostras) |
|---|---|---|
| sobrevive 20 s | 87% | **3/3 × 60 s** |
| ⟨\|ψ\|⟩ | 1,98° | **1,46°** (rms 1,9°, máx 8,85°) |
| ⟨\|x\|⟩ | 32 mm | **17,6 mm** |

Por episódio: ⟨|ψ|⟩ de 1,30°, 1,51° e 1,57°; ⟨|x|⟩ de 17,0, 17,8 e 17,9 mm.

**Deriva de −0,02, −0,16 e +0,14 mm/s** — zero dentro do ruído. Confirma a
etapa 6 por um caminho independente: sem perda de passos, a posição não
escorrega ao longo de 60 s.

`perdidos = 0` e CRC = 0 nos três. Um único `F_PERDEU` em 8.952 amostras, no
pico de 73 ms do episódio 1 — que **teria derrubado o episódio** com o
watchdog antigo de 60 ms.

Saturações: ação em 8,0 m/s² em 0,20% das amostras, velocidade em 0,450 m/s em
0,26%. Usa os limites, não vive nos limites.

### Dois ajustes que viabilizaram isto (29/10)

- **Política em numpy puro** no lugar do torch no laço quente. A conta é a
  mesma (confere em 2,4e−7), mas o torch pagava agendamento de threads e
  picava em 20 ms dentro de um laço de 20 ms. Latência: média 0,57 → **0,05
  ms**, pior caso 59× melhor.
- **`RL_TIMEOUT_MS` de 60 → 100 ms.** Os 60 ms eram 3 ciclos; o lote do
  USB-serial no macOS produz lacunas de até 53 ms no relógio do PC e o
  trânsito USB soma por cima. 100 ms fica abaixo do tempo de dobra do erro
  (105 ms), e com o supervisor ligado quem assume é o PID.

**Veredito da transferência:** as três métricas saíram **melhores** que a
previsão. A identificação do modelo (L_eq pelo período, atrito separado em
viscoso e de Coulomb) é o que pagou isso — o agente nunca viu esta bancada.

---

## Etapa 8 — Testes de perturbação

```bash
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
       --segundos 90 --log logs/rl_perturba_01.csv --auto --sem-observacao
```

**Supervisor LIGADO** nesta etapa (é o padrão). O ponto é medir a recuperação
do agente, não perder a haste no primeiro toque forte.

Com o agente equilibrando, dê **toquinhos leves** na haste com o dedo.
Comece muito suave e aumente aos poucos, esperando o pêndulo estabilizar entre
um toque e o outro (uns 3 s) para que cada perturbação seja medível em
separado.

O simulador prevê 90% de recuperação para chutes de 1,0 rad/s, e **zero**
acima de ~16° de inclinação — esse é o teto físico do atuador, não do agente.
Como o supervisor assume em 20°, o que você vai ver acima de ~16° é o PID
pegando, e isso é o resultado esperado, não falha.

Referência do regime sem perturbação, da etapa 7: ⟨|ψ|⟩ = 1,46°, |ψ| máximo
8,85°. Qualquer pico acima de ~9° no log é perturbação sua, e não o ciclo
limite — é assim que vou separar os eventos.

- [~] **PARCIAL em 29/10** (`logs/rl_perturba_01.csv`, 3278 amostras, 65,9 s).

  **Rejeição de perturbação: aprovada.** |ψ̇| chegou a **1,57 rad/s** em 1,89%
  das amostras — acima do chute de 1,0 rad/s que o simulador testou — e o
  ângulo nunca passou de **8,55°**, que é *menos* que o máximo de 8,85° sem
  perturbação alguma. Zero amostras acima de 9°. O agente absorveu os toques
  antes de virarem ângulo. O operador relatou "os toques não diferem do PID":
  não é falta de resposta, é ausência de drama.

  **Reprovado por posição.** Aos 65,9 s o carrinho fugiu para −233 mm e bateu
  no `X_LIM_RL`: `# SUPERVISOR: carrinho fora da faixa util`. A deriva
  acelerou de −6,8 mm/s (10 s) para −33,4 mm/s (3 s), com a velocidade
  saturada em −0,450 m/s. Falha de posição, não de ângulo.

  **Tremulação confirmada.** O operador relatou controle mais "nervoso" que o
  PID, e a medida concorda: **20,7 inversões de sinal da aceleração por
  segundo**, |Δa|/|a| = **1,14** — a ação se reinventa a cada ciclo.

### A causa de fundo: a recompensa não pagava por nada disso

Em regime permanente, com os valores medidos na bancada:

| Termo | Peso efetivo |
|---|---|
| sobrevivência | **1,000** |
| ψ (1,46°) | 0,001 |
| x (17,6 mm) | 0,001 |
| Δa (2,31) | 0,014 |

Os três termos de forma somavam 0,016 contra 1,000 de sobrevivência. O agente
era pago, na prática, só por não cair — e **tremular não custava nada**.

A coluna `suavid` do `avaliar.py` já media isso e marcava **1,724** para este
agente, contra 2,31 na bancada: o simulador previa a tremulação desde o
início e ninguém confrontou esse número com a bancada.

Correção aplicada em `pendulo_env.py` (anterior salvo em
`pendulo_env_v1.py.bak`):

- **Δa de 0,05 → 0,50**, linear. Passa a pesar 0,144 no nível antigo de
  tremulação contra 0,037 se suavizar — 0,107 por passo, mais de 100 de
  retorno por episódio. Linear e não quadrático de propósito: o gradiente
  quadrático desaparece justamente perto de zero, onde queremos empurrar.
- **Termo linear de posição** `0,15·|x|/x_lim` somado ao quadrático. O
  quadrático torna x pequeno quase gratuito, e o agente só reagia quando x já
  era grande — quando o limite de 0,45 m/s já impedia recuperar. Foi assim que
  o carrinho fugiu.

- [ ] **Aprovação (a refazer com o agente v2):** recupera de toques leves de
      forma visivelmente mais suave que o PID, sem fuga de posição em 90 s.

> **O que observar a olho nu.** O PID responde a um toque com uma correção
> brusca e uma oscilação que demora a sumir. Espera-se que o agente desloque o
> carrinho antes, com um movimento mais longo e menos abrupto. Se for o
> contrário — o agente mais nervoso que o PID — vale retreinar com o
> coeficiente de `|Δa|` maior, pelo mesmo motivo do achado da etapa 5.

---

## Resumo dos comandos

| Tecla | Função |
|---|---|
| `z` | define o zero (haste pendurada e parada) |
| `e` | centraliza e espera você levantar a haste |
| `i` | swing-up automático + PID |
| `r` | **entrega o controle ao agente** |
| `s` | para tudo |
| `m` | liga/desliga o supervisor PID (**só fora do modo RL**; use `--sem-supervisor`) |
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