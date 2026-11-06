# Pêndulo invertido: PID clássico e agente PPO embarcado

Pêndulo invertido físico (carro sobre trilho GT2, haste livre) com dois
caminhos de controle completos e independentes:

- **clássico** — swing-up por energia + PID, roda sozinho no ESP32
- **PPO** — um agente de aprendizado por reforço que faz swing-up *e*
  equilíbrio, treinado só em simulação e rodando **dentro do ESP32**

No caminho PPO não há PC no laço de controle e não há controle clássico.

## Resultado medido

| | clássico | PPO embarcado |
|---|---|---|
| swing-up | 3,55 s | **2,47 s** |
| ⟨\|ψ\|⟩ em regime | 1,90° | **0,75°** |
| em pé (\|ψ\|<10°) | — | **99,5%** |
| quedas | 0 (não recupera se cair) | nenhuma em 178 s; recupera em 0,81 s |
| ⟨\|x\|⟩ | 66 mm | 34 mm |
| \|x\| máx na subida | — | 134 mm de 240 |
| onde a decisão acontece | ESP32, 1 kHz | ESP32, 50 Hz |

Rede: 6→64→64→1 com tanh, 4 673 parâmetros, 18,7 kB em SRAM.
Inferência: **335 µs de p99 — 2,1% do passo de 20 ms.**
Conferência placa × PC: 5,4e-06 em 8 959 amostras.

O agente nunca viu o equipamento antes de ser ligado a ele. A concordância
simulador–bancada vem da identificação do modelo a partir de logs da própria
bancada, não de valores de catálogo.

---

## Hardware

| Componente | Especificação |
|---|---|
| Microcontrolador | **ESP32 Dev Module** (migração para ESP32-S3 DevKitC-1 N8R2 mapeada abaixo) |
| Motor | NEMA17, 200 passos/volta, 24 V |
| Driver | DRV8825, 8 micropassos |
| Encoder | incremental, 600 pulsos/volta, quadratura → 2400 contagens/volta = 0,15° |
| Transmissão | polia GT2 20 dentes, passo 2 mm |
| Haste | 255 mm do eixo à ponta, ~42 g, massa concentrada na ponta |
| Trilho | 600 mm de vão entre as chaves; carrinho de 65,5 mm → 534,5 mm de percurso |
| Segurança | duas chaves de fim de curso em interrupção + chave de emergência nos 24 V |

**A conversão passos→metros é exata, não medida:**
20 dentes × 2 mm = 40 mm/volta; 200 × 8 micropassos = 1600 pulsos/volta;
40 ÷ 1600 = 0,025 mm/pulso = **40 000 passos/m**. Três fontes independentes
concordam em 0,5%: aritmética (40 000, exata), dinâmica do swing-up
(39 200 ± 1200) e régua (40 196 ± 200). Por isso `CURSO_REAL_M = 0`.

### Ligações

| Sinal | ESP32 Dev Module | ESP32-S3 N8R2 | Liga em | Observação |
|---|---|---|---|---|
| STEP | 25 | 4 | DRV8825 STEP | pulso por timer a 40 kHz |
| DIR | 26 | 5 | DRV8825 DIR | `LOW` = lado A |
| EN | 27 | 6 | DRV8825 nENBL | `LOW` habilita |
| Fim de curso A | 32 | 7 | chave A, outro lado em GND | NF, `INPUT_PULLUP` |
| Fim de curso B | 33 | 15 | chave B, outro lado em GND | NF, `INPUT_PULLUP` |
| Encoder A | 18 | 16 | canal A | `attachInterrupt` em `CHANGE` |
| Encoder B | 19 | 17 | canal B | pull-up externo 4,7 k para 3,3 V |

VMOT em 24 V, lógica em 3,3 V, **GND comum** entre placa, driver e encoder.
O firmware escolhe o conjunto sozinho com `#if defined(CONFIG_IDF_TARGET_ESP32S3)`.

Três escolhas importam:

- **Fins de curso normalmente fechados.** Acionado é `HIGH` = contato aberto.
  Fio solto produz o mesmo sinal de chave acionada e para o carrinho — falha
  para o lado seguro.
- **Encoder em interrupção `CHANGE`** nos dois canais, decodificado por tabela
  de quadratura. É o que dá 2400 contagens de um encoder de 600 pulsos.
- **`nENBL` é ativo-baixo com pull-down interno**, então entre ligar os 24 V e
  o `setup()` rodar o driver fica habilitado e as bobinas energizadas. STEP e
  DIR também têm pull-down, logo o motor trava e não anda. Um pull-up de 10 k
  do EN para 3,3 V deixaria o driver desligado até o firmware assumir.

Nenhum dos sete pinos do S3 é strapping (0, 3, 45, 46), UART0 (43, 44),
USB (19, 20) nem o LED RGB (38). No S3, os sete ficam consecutivos no header
J1. Pelo conector USB nativo a porta vira `/dev/cu.usbmodem*`; pelo conector
UART continua `/dev/cu.usbserial-*`.

---

## Os arquivos

### Caminho clássico (não usa PC nem Python)

| Arquivo | O que é |
|---|---|
| `pend_rl/firmware/classico/pendulo_pid/pendulo_pid.ino` | swing-up por energia + PID, autossuficiente |

### Caminho PPO

| Arquivo | O que é |
|---|---|
| `pend_rl/pendulo_env.py` | simulador: a física identificada. **Única fonte dos parâmetros** |
| `pend_rl/pendulo_env_sw.py` | o mesmo com swing-up: 6 entradas, sin/cos, recompensa por altura |
| `pend_rl/ppo.py` | ator-crítico, buffer com GAE, atualização recortada |
| `pend_rl/treinar_sw.py` | treino do agente completo |
| `pend_rl/avaliar_sw.py` | seis baterias de validação em simulação |
| `pend_rl/exportar_c.py` | `.pt` → `politica.c`/`.h` + `oraculo.h` + `teste_politica.c` |
| `pend_rl/politica_c.py` | carrega `politica.c` via ctypes — o **mesmo** `.c` da placa |
| `pend_rl/comparar_c.py` | malha fechada torch × C, contra piso de ruído medido |
| `pend_rl/teste_cadeia.py` | regressão da cadeia de `a_ant` |
| `pend_rl/rl_bridge.py` | protocolo serial. Hoje **só leitura**, fora do laço |
| `pend_rl/conferir_local.py` | conferência bit a bit placa × PC (modo observador) |
| `pend_rl/registrar_local.py` | etapas 2 e 3 com o agente embarcado atuando |
| `pend_rl/modelos_sw/melhor.pt` | **o modelo embarcado** |
| `pend_rl/modelos_sw/agente.pt` | pesos da última iteração (ver "melhores resultados") |

### Firmware

| Pasta | O que é |
|---|---|
| `firmware/pendulo_rl/` | **o que está gravado**: `.ino` + `politica.c`/`.h` + `estado_rl.h`. Única cópia dos pesos |
| `firmware/politica/` | `oraculo.h` + `teste_politica.c`. Separado porque `teste_politica.c` tem `main()` e colidiria com o do core dentro de um sketch |
| `firmware/bench_politica/` | benchmark de latência isolado. O cabeçalho do `.ino` diz quais arquivos trazer |

Os pesos existem em **uma cópia só**, na pasta gravada. O teste e o `.so`
compilam contra ela.

---

## Executar o controle clássico

Autossuficiente: não precisa de PC além do monitor serial.

1. Gravar `pend_rl/firmware/classico/pendulo_pid/` (ESP32 Dev Module, 240 MHz).
2. Abrir o monitor serial a **115200**.
3. Esperar o homing e a mensagem de chegada ao meio.
4. Com a **haste pendurada e parada**, enviar `z` (zera o encoder).
5. Enviar `i` — swing-up por energia, entrega ao PID no topo.

Teclas: `i` swing-up+PID · `e` PID manual · `z` zera · `c` centraliza ·
`a`/`b` vai ao fim de curso · `p` imprime estado · `s` para.

Constantes: `KP` 40, `KD` 5,5, `KX` 5,0, `KV` 17,5, `A_BAL_MAX` 8,0 m/s²,
`ALVO_EN` 1,10, `A_SW_MAX` 4,0 m/s², `L_EQ_M` 0,220.

**Por que `ALVO_EN` tem de passar de 1,00.** Em exatamente 1,00 a trajetória é
a homoclínica: aproxima-se do topo assintoticamente e nunca o transpõe. Com
alvo 1,00, 28% das tentativas ficavam presas entre 0,90 e 1,00; com 1,10,
nenhuma.

**Por que `KX` = 5,0 e não 1,5.** Um offset de zero do encoder vira
deslocamento permanente, `x_ss ≈ −KP·offset/KX` — cerca de 47 mm por 0,1° com
`KX` 1,5. Elevar para 5,0 levou o deslocamento de 73 mm para 7 mm.

**Por que o PID roda a 1 kHz e o agente a 50 Hz.** O produto `KV·dt` é o que
limita a taxa: a 1 kHz, `KV·dt` = 0,0175; a 50 Hz seria 0,35 — realimentação
de velocidade grande demais por passo, que desestabiliza o laço discreto.

---

## Executar o agente PPO embarcado

1. Gravar `pend_rl/firmware/pendulo_rl/` (ESP32 Dev Module, **240 MHz**).
2. Monitor serial a **921600**. Confirmar no boot:
   `# Rede embarcada: 6->64->64->1, 4673 parametros, 18692 bytes em SRAM`
3. Com a haste **pendurada e parada**: `z`, depois `c`.
4. `l` para ligar a rede local. Ele imprime um teste a seco, sem mover nada.
   Com o carrinho no centro, o valor esperado deste modelo é
   `bruta=-0.194 -> -1.554 m/s2`. **O sinal é o que importa**: invertido
   significa `ENC_SINAL` ou entrada trocada na observação.
5. `g` — agente completo. Ele ergue a haste e a mantém.

Para parar: `s` não funciona em modo binário (o firmware só lê o pacote de
7 bytes). Aperte **EN** na placa ou a chave de emergência.

Depois de um reset os três alternadores voltam a desligado: `z` `c` `l` `g`.

### Com registro em CSV (o PC só observa)

```bash
source venv/bin/activate && cd pend_rl
python registrar_local.py --porta /dev/cu.usbserial-0001 --etapa 3 --seg 180 --csv et3.csv
```

O script leva os alternadores a um estado conhecido, entra no modo e grava.
Com `--deixar-rodando` ele sai **sem** mandar parar — é como se prova que o
PC não está no laço.

### Teclas do firmware de RL (921600)

`z` zera o encoder · `c` centraliza · `l` rede local, roda na placa ·
`o` observador, o PID atua e a rede só opina · `g` agente completo ·
`r` entrega após captura · `s` para · `m` supervisor PID ·
`v` limitador de velocidade

---

## O modelo físico

Três parâmetros extraídos de logs da própria bancada. São eles que explicam a
transferência simulador→realidade.

| Parâmetro | Valor | De onde saiu |
|---|---|---|
| `L_EQ` | 0,22463 m | período da oscilação livre, extrapolado para amplitude zero |
| atrito viscoso `b` | 0,06274 1/s | envelope de decaimento da amplitude |
| atrito de Coulomb `c` | 0,04984 rad/s² | envelope de decaimento da amplitude |

`L_eq` é o comprimento de um pêndulo simples **equivalente**, não o da haste:
massa distribuída mais massa na ponta, e o que importa é a combinação. Medir a
haste com régua daria o número errado. Conferência independente: a razão
`L_eq/ℓ` = 0,881 implica ~20 g na ponta e ~22 g na haste — 47% da massa na
extremidade, que é a descrição física da peça.

A massa de 42 g **não entra na dinâmica**: o motor de passo impõe a trajetória
do carrinho independentemente da reação do pêndulo. Por isso a equação é mais
simples que a do CartPole clássico.

### Método

1. **Período → `L_eq`.** O período depende da amplitude (integral elíptica
   `T(A) = T0·(2/π)·K(sin²(A/2))`). Medindo em 55 ciclos de 54,8° a 3,8° e
   extrapolando para amplitude zero, obtém-se `L_eq` sem interferência do
   atrito. Desvio de 0,8 ms entre os 55 ciclos.
2. **Envelope → atrito.** A taxa de decaimento separa viscoso (exponencial,
   R² 0,928) de Coulomb (linear, R² 0,960). Nenhum isolado basta; os dois
   juntos ajustam `dA/dt = −(b/2)·A − 2c/(π·ω₀)`.

**Três armadilhas que custam o resultado:**

- A janela de análise começa nas **passagens por zero**, onde ω é máximo. Num
  ponto de retorno ω ≈ 0 e a quantização de 0,15° domina a medida — iniciar
  ali dá erro de 45°.
- Um ajuste RMS direto sobre a oscilação inteira **não serve**: a deriva de
  fase passa a dominar o erro e o otimizador converge para atrito zero.
- A coluna `a_ms2` do log é a aceleração **pedida**. O firmware satura a
  velocidade em ±0,45 m/s, e durante o swing-up ela fica saturada 19–25% do
  tempo: nesses instantes o log diz 8 m/s² e a aceleração real é zero. Use
  `dv/dt` da coluna `v_ms`. Com `a_ms2`: erro de 44 a 105°. Com `dv/dt`:
  0,6 a 3,3°.

### Validação

| Teste | Erro |
|---|---|
| Oscilação livre, 7 janelas de 5 s | 0,37° médio |
| Swing-up com o carrinho em movimento | 0,6° a 3,3° |
| Reconstrução da ação do PID a partir do log | resíduo 0,060 m/s² |
| Taxa de divergência perto do topo | bate com a teoria |

A última linha merece explicação. O pêndulo invertido é instável com
τ = √(L/g) = **0,151 s**: qualquer erro **dobra a cada ~105 ms**. Validar em
malha aberta por mais de ~0,3 s é matematicamente impossível, não importa o
modelo. A validação possível é medir o tempo de dobra e confirmar que bate.

| Horizonte | Erro RMS |
|---|---|
| 0,02 s (um passo do agente) | 0,06° |
| 0,1 s | 0,30° |
| 0,3 s | 1,28° |
| 1,0 s | 54,2° |

### A perturbação que realmente importa

Testando cada fonte de incerteza isoladamente, com ganhos fixos:

| Perturbação | Faixa | Sobrevive |
|---|---|---|
| Comprimento `L` | ±8% | 100% |
| Atritos `b`, `c` | ×0,5 a ×1,8 | 100% |
| Ganho de aceleração | ±10% | 100% |
| Escala de posição | ±5% | 100% |
| Atraso de atuação | 1 ou 2 passos | 95% |
| **Offset de zero do ângulo** | **σ = 0,8°** | **10%** |

O offset de zero é devastador pelo motivo certo: se o controlador acredita que
a vertical está 0,8° deslocada, ele segura a haste nesse ângulo errado, isso
exige aceleração constante num sentido, e o carrinho corre até a parede.
É por isso que ele está na randomização de domínio e que a recompensa tem
termo de posição.

---

## Treinar um modelo

### Não existe base de dados

PPO é aprendizado por reforço, não supervisionado: **ele gera os próprios
dados** interagindo com o simulador. Não há conjunto de treino para rotular.

O que veio de dados reais são os **parâmetros do simulador** — três constantes,
extraídas de quatro logs com ~203 s no total. Depois disso o treino é sintético
e ilimitado: 2 000 iterações × 8 192 passos = **16,4 milhões de passos**, o
equivalente a **91 horas de bancada**, em 79 minutos de CPU.

Os logs originais da identificação não ficam no diretório de trabalho; estão no
histórico (`git show a3984c8:logs/livre_01.csv`). Só precisam ser refeitos se a
mecânica mudar.

### Passo a passo

```bash
source venv/bin/activate && cd pend_rl

# 1. treinar (~79 min para 2000 iteracoes)
python treinar_sw.py --saida modelos_sw --nucleos 1

# 2. avaliar em simulacao: 6 baterias
python avaliar_sw.py modelos_sw/melhor.pt

# 3. exportar para C
python exportar_c.py modelos_sw/melhor.pt
```

**`--nucleos 1` é o mais rápido.** 86% do tempo é a simulação física: um laço
Python sequencial de 20 sub-passos de 1 ms por passo do agente. Mais threads só
disputam. Acelerar exigiria vetorizar vários ambientes em paralelo.

O treino grava `historico.csv` com uma linha por iteração, e salva `melhor.pt`
(melhor avaliação) e `agente.pt` (última iteração).

### Hiperparâmetros, e a razão de cada um

| Parâmetro | Valor | Por que |
|---|---|---|
| rede | 2×64, tanh | 6 entradas; mais capacidade só aumenta a variância |
| γ | **0,995** | o swing-up leva ~175 passos e o prêmio vem no fim. Com 0,99 o horizonte efetivo é 100 passos e o início do bombeamento fica cego |
| λ (GAE) | 0,95 | compromisso usual viés/variância |
| ε (recorte) | 0,2 | valor do artigo original |
| entropia | **0,01** | há uma armadilha (girar sem parar) e um ótimo distante: precisa explorar mais que os 0,004 do agente de equilíbrio |
| coef. de valor | 0,5 | — |
| parada por KL | 0,02 | interrompe as épocas quando a política já mudou demais |
| recorte de gradiente | 0,5 | evita um lote atípico desestabilizar o treino |
| passos por lote | 8 192 | episódios longos (1 200 passos) |
| ação | gaussiana contínua, logσ aprendido e independente do estado | aceleração é contínua; discretizar perderia resolução |
| `--frac-topo` | 0,30 | 30% dos episódios começam no topo, para a habilidade de equilibrar não se perder enquanto ele aprende a subir |

### Observação e recompensa do agente completo

Seis entradas normalizadas:

| Índice | Grandeza | Normalização |
|---|---|---|
| 0 | `x` | ÷ 0,24 m |
| 1 | `v` | ÷ 0,45 m/s |
| 2 | `sin(ψ)` | — |
| 3 | `cos(ψ)` | — |
| 4 | `−ω` | ÷ 15,0 rad/s |
| 5 | ação anterior | ÷ 8 m/s² |

`sin`/`cos` em vez de `ψ/ψmax`: com a haste pendurada ψ = 180°, o que daria
**5,14** numa entrada que nunca passou de ±1 no treino — e `ψ/35°` é
descontínuo na volta completa (+180° e −80° são o mesmo estado físico e dariam
+5,14 e −5,14). `sin`/`cos` é contínuo e único em toda a rotação.

A **ação anterior** existe porque o firmware aplica a ação com um passo de
atraso, o que viola a hipótese de Markov: o estado atual não descreve o que já
está a caminho.

A escala de ω é **15 rad/s, não 6**: para transpor o topo a haste passa pelo
fundo a 13,5 rad/s. Pico medido em bancada: 14,23 rad/s — a escala de 15 não
saturou.

```
altura = (1 + cos ψ)/2                        0 pendurado, 1 em pe
r = altura
  − 0,10·(ψ̇/15)²        nao gire sem parar
  − 0,20·(|x|/0,24)²     fique perto do centro
  − 0,05·(|x|/0,24)      idem, termo linear
  − 0,05·|Δa|/8          nao trepide
  + bonus de precisao se |ψ| < 10° E |ψ̇| < 2,0 rad/s
r = −50  ao sair do trilho
```

Recompensa por **altura**, não por proximidade do equilíbrio: dá gradiente em
toda a rotação, e é o que faz o agente descobrir que precisa bombear energia.
Ele chegou sozinho à estratégia de quatro impulsos com inversão de direção, e
captura em `En` = 0,99 — mais cedo que o clássico, que persegue 1,10.

### Randomização de domínio

A cada episódio: `L_eq` ±8%, os dois atritos de 0,5× a 1,8×, ganho de
aceleração ±10%, zero do encoder ±0,8°, atraso de 1 ou 2 passos. O agente nunca
treina duas vezes no mesmo mundo, e por isso não aprende a explorar
imperfeições de um modelo específico.

### Como obter melhores resultados

**1. A métrica de seleção satura — corrija isso primeiro.**
O critério é `if frac_pe > melhor`, estritamente maior. `frac_pe` chegou a
**100,0% na iteração 150 de 2 000**, e das 80 avaliações do treino **58
marcaram 100%** sem nenhuma ser salva. O `melhor.pt` embarcado é a iteração
150: ~73 dos 79 minutos de treino não estão nele.

E o treino continuou melhorando no que a métrica já não media:

| avaliações | ψ em pé | ⟨\|x\|⟩ | subida |
|---|---|---|---|
| it 1–250 | 0,664° | 43,6 mm | 2,82 s |
| it 501–1000 | 0,586° | 45,5 mm | 2,98 s |
| it 1501–2000 | **0,568°** | 52,0 mm | **2,51 s** |

O desempate natural é `psi_pe` (menor é melhor) quando `frac_pe` empata.
Antes de trocar, compare `agente.pt` com `melhor.pt` no `avaliar_sw.py`,
olhando a **bateria 2** (quanto do trilho usa): o `agente.pt` tem ψ 17% melhor
e ⟨\|x\|⟩ 43% pior, e a margem de trilho na subida é o fator apertado.

**2. Penalizar `Δa` penaliza a recuperação.** Resultado negativo medido: subir
o peso de `Δa` para suavizar a ação levou a recuperação de empurrões de 1,0
rad/s de 65% para 40%. Suavidade e recuperação são o mesmo recurso — a
capacidade de mudar de aceleração rápido. Não troque uma pela outra sem medir.

**3. Posição e ângulo são um compromisso explícito.** Uma tentativa de
reequilibrar os pesos levou ⟨\|x\|⟩ de 29 para 65 mm, porque em x = 65 mm a
punição de posição (0,041) ficou menor do que a preguiça economizava em `Δa`
(0,063). **Calcule os valores numéricos dos termos no ponto de operação
pretendido antes de treinar**, não depois.

**4. Verifique se a recompensa tem armadilha antes de gastar 79 minutos.**
Numa versão, "girar sem parar" pontuava 1,923 contra 2,000 do equilíbrio
perfeito — 96% do prêmio por um comportamento inútil. A correção foi condicionar
o bônus de precisão a `|ψ̇| < 2,0 rad/s`. Imprima o valor da recompensa para
três ou quatro comportamentos-limite (pendurado parado, girando, em pé parado,
em pé trepidando) e confira a ordem.

**5. Truncamento não é terminação.** Um episódio acaba por dois motivos: o
pêndulo caiu (não há futuro, `V(s')` = 0) ou o limite de passos foi atingido
(estava equilibrado e **continuaria**). Tratar truncamento como terminação
ensina ao crítico que estados bons valem zero, e o erro contamina o
aprendizado para trás. No truncamento o bootstrap usa `V(s_T)`.

**6. A distribuição inicial define o que é recuperável.** O agente só recupera
estados parecidos com os que viu começar. Alargue-a se quiser robustez a
perturbações maiores.

**7. Mantenha o offset de zero na randomização.** É a única perturbação que
derruba um controlador de ganho fixo (10% de sobrevivência). O hardware real
sempre terá um.

### Referência a bater

| Fonte | ⟨\|ψ\|⟩ |
|---|---|
| PID original no simulador a 50 Hz | 5,68° |
| Melhor PID possível a 50 Hz, sem offset | 0,24° |
| PID na bancada real a 1 kHz | 1,90° |
| **PPO embarcado** | **0,75°** |

---

## Embarcar o modelo no ESP32

O modelo **não precisa ser reduzido**: 4 673 parâmetros são 18,3 KiB de
`.rodata` contra 4 MB de flash, e 4 544 MACs + 128 tanh contra 20 ms de
orçamento. Não entra TensorFlow Lite Micro nem ESP-DL — os pesos viram um
array `const` e a inferência são ~40 linhas de C.

### Passos e testes, em ordem crescente de custo

```bash
cd pend_rl

# 1. gerar o C
python exportar_c.py modelos_sw/melhor.pt

# 2. TESTE: o C reproduz o torch?          criterio: < 1e-5
cd firmware/politica
cc -O2 -Wall -Wextra -I../pendulo_rl -o teste_politica \
   teste_politica.c ../pendulo_rl/politica.c -lm && ./teste_politica
cd ../..

# 3. TESTE: fidelidade e malha fechada     criterio: variantes em C "ok"
python politica_c.py && python comparar_c.py && python teste_cadeia.py
```

**4. TESTE na placa, isolado.** Copie `politica.c`, `politica.h` e
`oraculo.h` para `firmware/bench_politica/`, grave, leia a 115200.
Critérios: `oraculo erro max < 1e-5` (se falhar, **pare** — nada mais do
sketch significa coisa alguma) e `p99 < 1000 µs`. Medido: 1,192e-06 e 335 µs.

**5. Gravar `firmware/pendulo_rl/`** a 921600 e fazer o teste a seco com `l`.

**6. TESTE observador — risco mecânico zero.** O PID atua, a rede só opina:

```bash
python conferir_local.py --porta /dev/cu.usbserial-0001 --seg 30 --csv conf.csv
```

Critério: `erro em bruta max < 1e-5`. Medido: 1,96e-06 em 1 494 amostras.
É o único teste que exercita a cadeia inteira — encoder → observação → rede →
ação — com sensores reais, e o último em que falhar é de graça.

**7. TESTE etapa 2 — o agente atua, com supervisor:**

```bash
python registrar_local.py --porta ... --etapa 2 --seg 60 --csv et2.csv
```

Levante a haste à mão. Critério: `intervencoes 0`, `em pe > 95%`,
`|x| max < 230 mm`. **Ignore ⟨\|x\|⟩ dos primeiros 6 s**: o PID entrega a haste
a até 10° e o agente faz uma excursão grande para agarrá-la. Olhe o regime —
medido: banda instantânea de 1,46 mm, ψ = −0,163° ± 0,084°.

**8. TESTE etapa 3 — agente completo, sem controle clássico:**

```bash
python registrar_local.py --porta ... --etapa 3 --seg 180 --csv et3.csv
```

Centralize antes. Haste pendurada e livre, **mão na chave de emergência** —
aqui o firmware para em vez de devolver ao PID. Critério: `flags 164` em 100%
das amostras, subiu, `em pe > 85%`, `|x| max < 230 mm`.

### Decisões de implementação

- **Não use a LUT de tanh.** Os 128 `tanhf` são 70% dos 324 µs (425 ciclos
  cada, contra 5,1 por MAC), mas o gargalo inteiro cabe 60 vezes no orçamento.
  A LUT economizaria 215 µs de um orçamento com 19 600 sobrando, em troca de
  1000× menos precisão. Ela existe atrás de `POLITICA_TANH_LUT`, desligada.
- **Copie os pesos para SRAM no boot.** `const` vive em `.rodata`, atrás do
  cache de flash; um miss no meio da inferência é jitter.
- **Não use PSRAM** (no S3): é mais lenta que a SRAM interna e não resolve nada.
- **`#pragma GCC optimize ("O2")`** no `politica.c`: o core do ESP32 compila com
  `-Os`, e a IDE não expõe a opção.
- **A observação é montada na placa a partir dos valores transmitidos**, lidos
  num instantâneo único por fronteira de 20 ms. Sem isso a ISR pode alterar
  `posicao` entre a leitura do pacote e a da observação, e a comparação acusa
  diferença onde não há bug.
- **O atraso de um passo é preservado de graça**: a rede escreve em
  `aRL_pendente` e a máquina existente só promove na fronteira seguinte. É o
  mesmo atraso que o simulador modelou.
- **A ISR do gerador de passo é o custo oculto.** Com ela a 40 kHz, a
  inferência vai de 335 para 418 µs de p99 — ~4,9 µs por interrupção, ou ~20%
  de um núcleo. A causa provável é `digitalWrite`/`digitalRead` não estarem em
  IRAM no core do Arduino: trocar por acesso direto a registrador
  (`GPIO.out_w1ts`/`out_w1tc`) é a otimização mais barata que resta.

---

## Camadas de proteção

| Camada | Onde | O que faz |
|---|---|---|
| Supervisor PID | firmware | \|ψ\| > 20° → o PID retoma (tem até 35° para salvar) |
| Sanidade da rede | firmware | saída não finita ou \|bruta\| > 100 → `PARADO` |
| Saturações e freio de parede | firmware | velocidade, aceleração e curso |
| Fins de curso | ISR | cortam os pulsos na interrupção |
| Chave de emergência | o operador | em série com os 24 V |

As três primeiras são software e podem ter bugs. As duas últimas, não.

No agente completo (`g`) o supervisor de ângulo é **desligado**, porque durante
o swing-up \|ψ\| passa por 180° e ele dispararia no primeiro ciclo. O que resta
não é controle, é limite físico: nada ali calcula uma ação, apenas impede que a
do agente ultrapasse o que o hardware aguenta. E a saída por falha vai para
`PARADO`, não para o PID — entregar uma haste possivelmente pendurada a um PID
de equilíbrio seria pior que parar.

**Clássico e agente nunca atuam ao mesmo tempo.** Não há mistura de sinais nem
média ponderada: o controle é posse exclusiva. O comando `r` não entrega na
hora, arma uma intenção; a troca só acontece com \|ψ\| < 3°, \|ψ̇\| < 0,5 rad/s
e \|x\| < 120 mm ao mesmo tempo. Entregar um pêndulo já caindo seria pôr o
agente num estado que ele mal viu no treino.

---

## O que está só no histórico

O diretório guarda o necessário para operar e evoluir. As etapas anteriores
ficaram nos commits:

| o que | onde |
|---|---|
| logs brutos da identificação | `git show a3984c8:logs/livre_01.csv` |
| scripts da identificação | `git show 1bfbf23:pend_rl/analise/` |
| agente v1, só equilíbrio | `git show 1bfbf23:pend_rl/modelos/melhor.pt` |
| registro da validação embarcada (4 CSVs) | `git show a3984c8:pend_rl/logs/embarcado/` |
| corridas de bancada da era da ponte | `git show a3984c8:pend_rl/logs/` |
| testes da ponte-no-laço | `git show 1bfbf23:pend_rl/teste_auto.py` |
| firmware clássico original | `git show 1bfbf23:testependulo/testependulo.ino` |
