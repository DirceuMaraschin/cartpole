# Pêndulo invertido: PID clássico e agente PPO embarcado

Pêndulo invertido físico (carro sobre trilho GT2, haste livre) com dois
caminhos de controle completos e independentes:

- **clássico** — swing-up por energia + PID, roda sozinho no ESP32
- **PPO** — um agente de aprendizado por reforço que faz swing-up *e*
  equilíbrio, treinado só em simulação e rodando **dentro do ESP32**

No caminho PPO não há PC no laço de controle e não há controle clássico.

## Resultado

Duas versões do agente foram construídas e medidas. A segunda roda embarcada.

| | clássico | agente 1 (equilíbrio) | agente 2 embarcado |
|---|---|---|---|
| Quem faz o swing-up | energia, 3,55 s | energia, 3,55 s | **o próprio agente, 2,47 s** |
| Quem equilibra | PID | o agente | o agente |
| Controle clássico na solução | tudo | energia + PID de rede | **nenhum** |
| Onde a decisão acontece | ESP32, 1 kHz | PC, 50 Hz | **ESP32, 50 Hz** |
| ⟨\|ψ\|⟩ em regime | 1,90° | 1,46° | **0,75°** |
| em pé (\|ψ\|<10°) | — | — | **99,5%** |
| ⟨\|x\|⟩ | 66 mm | 17,6 mm | 34 mm |
| quedas | 0 (não recupera) | 0 em 6,2 min | nenhuma em 178 s |
| recuperação de queda | impossível | impossível | **0,81 s** |

Rede: 6→64→64→1 com tanh, 4 673 parâmetros, 18,7 kB em SRAM.
Inferência: **335 µs de p99 — 2,1% do passo de 20 ms.**
Conferência bit a bit placa × PC: 5,4e-06 em 8 959 amostras.

### A concordância simulador–bancada

É o resultado central do projeto. O agente nunca viu o equipamento antes de ser
ligado a ele, e as métricas previstas saíram **melhores** na bancada:

| Métrica | Simulador previu | Bancada |
|---|---|---|
| ⟨\|ψ\|⟩ (agente 1) | 1,98° | **1,46°** |
| ⟨\|x\|⟩ (agente 1) | 32 mm | **17,6 mm** |
| sobrevive 20 s | 87% | 3 de 3 episódios de 60 s |
| swing-up até capturar | 3,60 s | 3,55 s |
| swing-up do agente 2 | 2,87 s | **2,47 s** |

Isso vem da identificação do modelo a partir de logs da própria bancada, não de
valores de catálogo.

---

## O hardware

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
O firmware escolhe o conjunto sozinho com `#if defined(CONFIG_IDF_TARGET_ESP32S3)`,
e o mesmo arquivo compila para as duas placas.

Três escolhas importam para a segurança:

- **Fins de curso normalmente fechados.** Acionado é `HIGH`, ou seja contato
  **aberto**. Um fio solto produz o mesmo sinal de uma chave acionada e para o
  carrinho — falha para o lado seguro. O custo é que um fio solto parece
  defeito de lógica, e foi exatamente o que aconteceu uma vez na bancada.
- **Os dois canais do encoder entram por interrupção em `CHANGE`**,
  decodificados por tabela de quadratura. É o que dá 2400 contagens por volta
  a partir de um encoder de 600 pulsos.
- **`nENBL` do DRV8825 é ativo-baixo com pull-down interno**, então enquanto o
  GPIO do EN flutua — entre ligar a alimentação e o `setup()` rodar — o driver
  fica habilitado e as bobinas energizadas. STEP e DIR também têm pull-down,
  logo não há pulsos espúrios: o motor trava, não anda. Um pull-up de 10 k do
  EN para 3,3 V deixaria o driver desligado até o firmware assumir. No S3 a
  janela é maior, porque a enumeração USB atrasa o boot.

### Migração para o ESP32-S3 DevKitC-1 N8R2

**Nenhum dos sete pinos sobrevive à troca de placa.** As restrições do S3:

| Faixa | Situação |
|---|---|
| GPIO 22–25 | **não existem** no chip |
| GPIO 26–32 | barramento da flash SPI |
| GPIO 33, 34 | não saem nos headers do DevKitC-1 |
| GPIO 35–37 | livres no N8R2; a reserva vale só para WROOM-2 (N16R8V, N32R8V) |
| GPIO 19, 20 | USB D− e D+ do conector nativo |
| GPIO 43, 44 | UART0 |
| GPIO 0, 3, 45, 46 | strapping |
| GPIO 38 | LED RGB da placa (v1.1) |

Dos pinos antigos: 25 não existe; 26, 27 e 32 são da flash; 33 não sai no
header; 19 é USB D−. Só o 18 seria aproveitável, e separá-lo do par do encoder
não ajudaria. Nenhum dos sete novos é strapping, UART0, USB ou LED RGB, e eles
ficam consecutivos no header **J1**: um único cabo de fita atende todos.

**O que mais muda com o S3:**

- **A porta serial troca de nome.** O DevKitC-1 tem dois conectores USB-C. Pelo
  conector **UART** (ponte USB-serial) a porta continua `/dev/cu.usbserial-*`;
  pelo conector **USB** nativo ela vira `/dev/cu.usbmodem*`. O `--porta` dos
  scripts precisa acompanhar.
- **O USB nativo pode resolver a latência.** As lacunas de 45 a 53 ms que
  forçaram o watchdog de 60 para 100 ms vinham do lote da ponte USB-serial. O
  conector nativo é CDC direto. Com a rede embarcada isso deixou de estar no
  laço de controle, então hoje é conveniência, não necessidade.
- **Configuração na IDE:** placa `ESP32S3 Dev Module`, Flash 8 MB, PSRAM
  `QSPI PSRAM`, e `USB CDC On Boot` ligado se usar o conector nativo.
- **O gerador de pulsos não muda.** O firmware já ramifica entre as APIs de
  timer dos cores 2.x e 3.x, e o S3 roda os mesmos 40 kHz.

Antes do primeiro episódio com a placa nova, refaça a **etapa 0**: o controle
clássico tem de funcionar como antes. Uma inversão de fio no encoder ou no DIR
aparece lá, não depois.

**O S3 compra pouco neste escopo.** As instruções vetoriais dele são de
inteiro, 8 e 16 bits — é por isso que ESP-NN e ESP-DL aceleram redes
quantizadas. Em float32 não há ganho, e este modelo não precisa de quantização.
O S3 passaria a se justificar com rede bem maior, quantização int8/int16 para
valer, ou telemetria sem fio por BLE 5.

### A aritmética da transmissão é exata

A conversão de passos para metros não foi medida, foi calculada:

- 20 dentes × 2 mm de passo = **40 mm por volta** da polia
- 200 passos × 8 micropassos = **1600 pulsos por volta**
- 40 mm ÷ 1600 = **0,025 mm por pulso** = **40 000 passos por metro**

Exato por construção, e por isso prevalece sobre qualquer medida de régua. Três
caminhos independentes concordam dentro de 0,5%: a aritmética (40 000, exata),
a dinâmica do swing-up (39 200 ± 1200) e a régua (40 196 ± 200). Por isso
`CURSO_REAL_M = 0`.

A divergência que existia era de medida, não de hardware: a régua media o **vão
livre** entre as chaves (600,0 mm) e o firmware mede o **percurso do carrinho**
(600,0 − 65,5 = 534,5 mm). Ao tocar a chave A uma face do carrinho está nela;
ao tocar a B, é a outra.

O encoder dá 600 × 4 = **2400 contagens por volta**, ou **0,15° por contagem**.
Essa quantização é o piso de precisão de tudo o que vem depois: nenhuma medida
de ângulo no projeto pode ser mais fina que meio passo, 0,075°.

---

## O modelo físico

Três parâmetros extraídos de logs da própria bancada. São eles que explicam a
transferência simulador→realidade.

| Parâmetro | Valor | De onde saiu |
|---|---|---|
| `L_EQ` | 0,22463 m | período da oscilação livre, extrapolado para amplitude zero |
| atrito viscoso `b` | 0,06274 1/s | envelope de decaimento da amplitude |
| atrito de Coulomb `c` | 0,04984 rad/s² | envelope de decaimento da amplitude |

Os dados de partida, 203 s em quatro logs:

| Arquivo | Duração | Conteúdo | Para que serviu |
|---|---|---|---|
| `livre_01.csv` | 72,4 s | oscilação livre, carrinho parado | `L_eq` e atrito |
| `pid_01.csv` | 41,3 s | PID equilibrando | conferência da lei de controle, ruído, atraso |
| `swing_01.csv` | 51,0 s | swing-up + equilíbrio | acoplamento carrinho→haste |
| `swing_02.csv` | 38,6 s | idem | validação cruzada |

`dt` medido de 10,04 ms com desvio de 0,049 ms e **zero amostras perdidas** nos
quatro arquivos: o registrador não interferiu no controle.

`L_eq` é o comprimento de um pêndulo simples **equivalente**, não o da haste:
massa distribuída mais massa na ponta, e o que importa para a dinâmica é a
combinação. Medir a haste com régua daria o número errado.

Conferência física independente: a razão `L_eq/ℓ` = 0,881 implica
`m_ponta ≈ 20 g` e `m_haste ≈ 22 g` — **47% da massa na extremidade**, que é a
descrição física da peça. Dois caminhos distintos (período de oscilação ×
geometria e massa) chegam ao mesmo número.

A massa de 42 g **não entra na dinâmica**: o motor de passo impõe a trajetória
do carrinho independentemente da reação do pêndulo, e por isso a equação é mais
simples que a do CartPole clássico. A massa só importaria para conferir torque:
42 g pedem ~0,4 N contra os ~39 N que o NEMA 17 entrega pela polia.

### Método

Os dois efeitos vivem em escalas de tempo diferentes e foram separados:

1. **Período → `L_eq`.** O período depende da amplitude:

```latex
T(A) = T_0 \cdot \frac{2}{\pi} \cdot K\!\left(\sin^2(A/2)\right)
```

   Medindo 64 cruzamentos por zero em `livre_01.csv`, extraindo o período de
   cada ciclo com sua amplitude e corrigindo pela integral elíptica, obtém-se
   `T0` à amplitude zero: **0,9508 s** com desvio de 0,8 ms entre 55 ciclos de
   54,8° a 3,8°. Daí `ω₀` = 6,6085 rad/s e `L_eq` = 224,63 mm. O firmware tinha
   220,0 mm — erro de +2,1%.

2. **Envelope → atrito.** A forma do decaimento diz o tipo: exponencial
   (viscoso puro) ajusta com R² 0,928; linear (Coulomb) com R² 0,960. Nenhum
   isolado basta, então os dois juntos:

```latex
\frac{dA}{dt} = -\frac{b}{2}A - \frac{2c}{\pi \omega_0}
```

O Coulomb é implementado como `c·tanh(ω/0,02)` em vez de `c·sign(ω)`, porque
`sign` é descontínuo e gera trepidação numérica com a haste quase parada.

**Três armadilhas que custam o resultado:**

- A janela de análise começa nas **passagens por zero**, onde ω é máximo. Num
  ponto de retorno ω ≈ 0 e a quantização de 0,15° domina a estimativa de
  velocidade — iniciar ali dá erro de 45°. Não é falha do modelo: é a fase
  inicial saindo errada.
- Um ajuste RMS direto sobre a oscilação inteira **não serve**: a deriva de
  fase passa a dominar o erro e o otimizador converge para atrito zero.
- A coluna `a_ms2` do log é a aceleração **pedida**. O firmware satura a
  velocidade em ±0,45 m/s, e durante o swing-up ela fica saturada 19–25% do
  tempo: nesses instantes o log diz 8 m/s² e a aceleração real é **zero**. Use
  `dv/dt` da coluna `v_ms`. Com `a_ms2`: erro de 44 a 105°. Com `dv/dt`:
  0,6 a 3,3°.

### Validação

| Teste | Erro |
|---|---|
| Oscilação livre, 7 janelas de 5 s | 0,37° médio (0,10° a 0,85°) |
| Swing-up com o carrinho em movimento, 5 janelas | 0,57° a 1,60° |
| Reconstrução da ação do PID a partir do log | resíduo 0,060 m/s² |
| Taxa de divergência perto do topo | bate com a teoria |

A reconstrução merece nota: recompondo `a = 40ψ + 5,5ψ̇ + 1,5x + 17,5v` das
colunas do log, o resíduo médio é −0,174 m/s² com desvio 0,060, e **100% das
amostras caem dentro de ±0,5** — que é exatamente o limite `I_MAX`. O resíduo
*é* o termo integral, o que prova que a interpretação das colunas e da lei de
controle está exata.

A última linha da tabela também. O pêndulo invertido é instável com
τ = √(L/g) = **0,151 s**: qualquer erro **dobra a cada ~105 ms**.

| Horizonte | Erro RMS | Fator |
|---|---|---|
| 0,02 s (um passo do agente) | 0,06° | — |
| 0,1 s | 0,30° | — |
| 0,2 s | 0,70° | 2,3× |
| 0,3 s | 1,28° | 1,8× |
| 0,5 s | 3,90° | 3,0× |
| 1,0 s | 54,2° | 14× |

Isso **confirma** o modelo em vez de contestá-lo: a taxa de divergência medida
bate com a instabilidade teórica. Validar um pêndulo invertido por simulação em
malha aberta por mais de ~0,3 s é matematicamente impossível, não importa quão
bom seja o modelo.

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

**Isso aconteceu na bancada.** Em `pid_01.csv` o `x` tem média de +175 mm e
chega a 256 mm. Não é defeito do PID: é a calibração do zero. O antídoto é
realimentação de posição mais forte:

| `KX` | Sobrevive com offset |
|---|---|
| 0,44 | 10% |
| 1,5 | 25% |
| 5 | 65% |
| **8** | **85%** |
| 12 | 30% |
| 18 | 0% |

Por isso o offset está na randomização de domínio e a recompensa tem termo de
posição: o hardware real sempre terá um, e o agente precisa aprender a tolerá-lo.

### A descoberta sobre a taxa de controle

Rodando o PID com os ganhos exatos do firmware dentro do simulador:

| Taxa | Sobrevive 20 s | ⟨\|ψ\|⟩ |
|---|---|---|
| 1000 Hz | 20/20 | 0,29° |
| 200 Hz | 20/20 | 0,38° |
| 100 Hz | 20/20 | 4,56° |
| 50 Hz | 20/20 | 5,70° |
| 25 Hz | 0/20 | falha |

Parecia que 50 Hz seria limitante demais para o agente. **Não é** — bastam
ganhos próprios para 50 Hz:

| Conjunto | KP | KD | KX | KV | ⟨\|ψ\|⟩ a 50 Hz |
|---|---|---|---|---|---|
| sintonizado a 1 kHz | 40 | 5,5 | 1,5 | 17,5 | 5,68° |
| **reajustado para 50 Hz** | 41,1 | 4,51 | 0,44 | **1,3** | **0,24°** |

O culpado é o `KV`. A 1 kHz, `KV·dt` = 0,0175; a 50 Hz, 0,35 — realimentação de
velocidade grande demais por passo, que desestabiliza o laço discreto. O PPO
descobre sozinho ganhos adequados ao seu próprio `dt`, e essa é justamente uma
vantagem dele sobre transplantar ganhos de outro regime.

### O que o simulador replica do hardware

- integração interna a 1 kHz, com decisão a 50 Hz (igual ao firmware)
- quantização do encoder em 0,15°
- o mesmo filtro de ω: janela de 8 amostras + EMA de 50%
- saturação de velocidade e freio de parede, com as mesmas constantes
- atraso de atuação de 1 ou 2 passos

---

## Controle clássico: a linha de base

Dois controladores em sequência, um para erguer a haste e outro para mantê-la.
Ambos continuam no firmware de RL: como linha de base de comparação e como rede
de proteção do agente 1.

### Swing-up por energia

Ergue a haste bombeando energia a cada meio ciclo. A grandeza perseguida é a
energia normalizada:

```latex
E_n = \tfrac{1}{2}\left(\frac{\omega}{\omega_0}\right)^2 - \cos\varphi,
\qquad \omega_0 = \sqrt{g/L_{eq}}
```

`E_n` vale −1 com a haste pendurada e parada, e +1 no topo. O controlador
acelera o carrinho no sentido que aumenta `E_n` até o alvo, e entrega ao PID
quando a haste chega perto do topo.

**O alvo precisa ultrapassar 1,00.** Em exatamente 1,00 a trajetória é a
homoclínica — a que se aproxima do topo assintoticamente e nunca o transpõe.
Com alvo em 1,00, 28% das tentativas ficavam presas entre `E_n` 0,90 e 1,00.
Com `ALVO_EN = 1,10`, nenhuma.

O `ω₀` usado aqui é **parâmetro de sintonia**, não afirmação sobre a física: o
firmware usa `L_EQ_M = 0,220`, enquanto a identificação deu 0,22463.

### PID de equilíbrio

```latex
a = K_P\,\psi + K_D\,\dot{\psi} + K_X\,x + K_V\,v
```

com `KP` = 40, `KD` = 5,5, `KX` = 5,0, `KV` = 17,5. Os dois primeiros termos
equilibram a haste; os dois últimos centralizam o carrinho, e sem eles ele
deriva até a parede.

Dois comportamentos importam para o resto do projeto:

- **`KX` determina o erro de posição em regime.** Um offset de zero do encoder
  vira deslocamento permanente, `x_ss ≈ −KP·offset/KX` — cerca de 47 mm por
  0,1° com `KX` 1,5. Elevar para 5,0 levou o deslocamento de 73 mm para 7 mm.
- **O produto `KV·dt` é o que limita a taxa.** A 1 kHz, `KV` = 17,5 é estável;
  a 50 Hz o mesmo ganho desestabiliza. Por isso o PID roda a 1 kHz no firmware
  enquanto o agente decide a 50 Hz.

### O que a linha de base entrega

| Métrica | Valor medido |
|---|---|
| swing-up até capturar | 3,55 s |
| capturas perdidas | 0 |
| ⟨\|ψ\|⟩ em regime | 1,90° |
| ⟨\|x\|⟩ em regime | 66 mm |

Estes são os números que o PPO precisa bater — e o controlador que assume
quando o agente 1 falha.

### Executar o controle clássico

Autossuficiente: não precisa de PC além do monitor serial.

1. Gravar `pend_rl/firmware/classico/pendulo_pid/` (ESP32 Dev Module, 240 MHz).
2. Abrir o monitor serial a **115200**.
3. Esperar o homing e a mensagem de chegada ao meio.
4. Com a **haste pendurada e parada**, enviar `z` (zera o encoder).
5. Enviar `i` — swing-up por energia, entrega ao PID no topo.

Teclas: `i` swing-up+PID · `e` PID manual · `z` zera · `c` centraliza ·
`a`/`b` vai ao fim de curso · `p` imprime estado · `s` para.

Constantes: `KP` 40, `KD` 5,5, `KX` 5,0, `KV` 17,5, `A_BAL_MAX` 8,0 m/s²,
`ALVO_EN` 1,10, `A_SW_MAX` 4,0 m/s², `KX_SW` 1,0, `KV_SW` 0,1, `L_EQ_M` 0,220.

São as mesmas constantes do `pendulo_rl.ino`: o firmware de RL não tem física
diferente, tem o modo RL a mais.

---

## PPO: como funciona

O PPO treina duas redes ao mesmo tempo. O **ator** transforma o estado
observado em uma ação; o **crítico** estima quanto retorno esperar daquele
estado, e serve para julgar se a ação tomada foi melhor ou pior que a média.

A ideia central está no **objetivo recortado**. Em vez de empurrar a política o
quanto o gradiente sugerir, ele limita o quanto a probabilidade de uma ação
pode mudar por atualização:

```latex
L^{CLIP}(\theta) = \mathbb{E}\left[\min\left(r_t(\theta)\,\hat{A}_t,\
\mathrm{clip}\big(r_t(\theta),\,1-\epsilon,\,1+\epsilon\big)\,\hat{A}_t\right)\right]
```

onde `rₜ` é a razão entre a probabilidade da ação na política nova e na antiga,
e `Âₜ` é a vantagem estimada. O recorte em 1 ± ε é o que impede uma atualização
grande de destruir uma política que já funcionava — o modo de falha clássico
dos métodos de gradiente de política anteriores.

A vantagem vem do GAE, que combina estimativas de vários horizontes:

```latex
\hat{A}_t = \sum_{l=0}^{T-t-1} (\gamma\lambda)^l \delta_{t+l},
\qquad \delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
```

### A armadilha do truncamento

É a sutileza que silenciosamente arruína implementações de PPO. Um episódio
pode acabar por dois motivos distintos:

- **Terminação** — o pêndulo caiu. Não há futuro, e o valor do estado seguinte
  é genuinamente zero.
- **Truncamento** — o limite de passos foi atingido. O pêndulo estava
  equilibrado e **continuaria**: o futuro existe, só não foi simulado.

Tratar truncamento como terminação ensina ao crítico que estados perfeitamente
bons valem zero, e o erro contamina o aprendizado para trás. No truncamento o
bootstrap tem de usar `V(s_T)` do crítico; na terminação, zero.

---

## Os dois agentes

### Agente 1: equilíbrio

Recebe a haste já erguida pelo controlador de energia e a mantém. Cinco
entradas normalizadas:

| Índice | Grandeza | Normalização | Por quê |
|---|---|---|---|
| 0 | `x` | ÷ 0,24 m | limite de software |
| 1 | `v` | ÷ 0,45 m/s | saturação do atuador |
| 2 | `ψ` | ÷ 35° | limite do episódio |
| 3 | `ψ̇` | ÷ 6,0 rad/s | valor empírico dos logs |
| 4 | ação anterior | ÷ 8 m/s² | **compensa o atraso de atuação** |

A quinta entrada não existe no CartPole padrão. Ela entra porque o atraso de um
passo viola a hipótese de Markov: o estado atual não descreve o que já está a
caminho. Dando a última ação ao agente, a informação volta a ser completa.

```latex
r = 1 - 0{,}60\left(\frac{|\psi|}{\psi_{max}}\right)^2
      - 0{,}25\left(\frac{|x|}{x_{max}}\right)^2
      - 0{,}05\frac{|\Delta a|}{a_{max}}
```

com −50 ao cair ou sair do trilho, e término em |ψ| > 35°, |x| > 0,24 m ou
1000 passos (truncamento, sem punição).

Os três termos de forma somam cerca de 0,016 em regime contra 1,000 da
sobrevivência: na prática o agente é pago por não cair, e o custo de uma ação
nervosa é quase nulo. Isso apareceu na bancada como 20,7 inversões de sinal da
aceleração por segundo.

**Foi validado em nove etapas**, cada uma com critério definido *antes* de rodar:

| Etapa | O que testa | Critério | Resultado |
|---|---|---|---|
| 0 | o PID continua funcionando após a recompilação | swing-up captura | 3,6 s na primeira tentativa |
| 1 | velocidade em 100% | fator 1,0 | `vmax=0,450` confirmado |
| 2 | ponte e simulador veem o mesmo estado | erro < meio passo | x 0,0125 mm, ψ 0,063° |
| 3 | o agente decide, o PID atua | mesmo sinal > 60%, correlação > 0,3 | **80% e +0,916** |
| 4 | primeiro episódio real | 10 s sem supervisor | 30 s, zero intervenções |
| 5 | supervisor desligado no firmware | 20 s sozinho | 20 s, ⟨\|ψ\|⟩ 1,53° |
| 6 | perda de passos | sem deriva após 3 episódios | nenhuma |
| 7 | desempenho real | comparar com o simulador | 3 × 60 s completos |
| 8 | perturbação | recupera de toques leves | 24 de 24 toques |

**A etapa 3 é a mais valiosa, e nada se move nela.** O agente decide e o PID
atua. Mesmo assim ela responde à pergunta mais perigosa do projeto: *o sinal
está invertido em algum ponto da cadeia?* Se `ENC_SINAL`, a convenção de ψ ou a
direção do motor estiverem trocados, o agente decidiria sistematicamente no
sentido contrário ao do PID e empurraria o carrinho para o lado da queda. A
correlação de **+0,916** descarta isso antes de qualquer movimento comandado
pela rede.

Resultado: **6,2 minutos** de equilíbrio com **zero** intervenções do
supervisor, 8 952 amostras, um único pacote perdido. Deriva de posição nos três
episódios: −0,02, −0,16 e +0,14 mm/s — zero dentro do ruído, o que confirma a
etapa 6 por um caminho independente.

### Agente 2: swing-up e equilíbrio, sem controle clássico

Recebe a haste pendurada e parada, ergue-a sozinho e a mantém. Não há PID nem
controlador de energia em momento algum. **É este que está embarcado.**

Quatro mudanças em relação ao agente 1, cada uma obrigatória:

**1. O ângulo passa a ser `sin(ψ)` e `cos(ψ)`.** A observação antiga entregava
`ψ/35°`. Com a haste pendurada ψ = 180°, o que dá **5,14** numa entrada que
nunca passou de ±1 no treino. Pior: `ψ/35°` é descontínuo na volta completa —
+180° e −80° são o mesmo estado físico e dariam +5,14 e −5,14. `sin`/`cos` é
contínuo e único em toda a rotação. Como a rede passa de 5 para 6 entradas,
isso obriga a treinar do zero.

**2. Sem terminação por ângulo.** Antes o episódio morria com −50 quando |ψ|
passava de 35°: a região do swing-up era, para o agente, literalmente o fim do
mundo. Agora só o trilho termina o episódio.

**3. Recompensa por altura, não por proximidade do equilíbrio.**
`(1 + cos ψ)/2` vale 0 pendurado e 1 em pé, e dá gradiente em toda a rotação —
é o que faz o agente descobrir que precisa bombear energia.

**4. Escala de `ψ̇` de 6 para 15 rad/s.** Para transpor o topo a haste passa
pelo fundo a 13,5 rad/s. Em 6,0 a observação saturaria 2,3× durante toda a
subida. O pico medido em bancada foi 14,23 rad/s — a escala de 15 não saturou.

**Como ele sobe.** Quatro impulsos, com inversão de direção do carrinho a cada
um, usando 134 mm de curso (254 mm na versão com a ponte). No quarto, a energia
ultrapassa o topo e o agente a captura exatamente em `E_n` = **0,99** — energia
de estar em pé e parado. O controlador clássico persegue `ALVO_EN` = 1,10 por
sintonia manual; o agente chegou à mesma estratégia sozinho, e captura mais
cedo.

**Cair deixou de ser falha.** Na versão com a ponte ele caía cerca de uma vez a
cada 45 s, dava a volta completa e refazia o swing-up em 0,81 s. Em simulação
recupera **95%** dos chutes de 3,0 rad/s, contra **0%** do agente 1 a 1,5 rad/s.
Não é um controlador melhor perto do equilíbrio — é um controlador para quem
perder a haste é recuperável. Embarcado, com a latência da serial fora do laço,
não houve nenhuma queda em 178 s.

---

## Treinar um modelo

### Não existe base de dados

PPO é aprendizado por reforço, não supervisionado: **ele gera os próprios
dados** interagindo com o simulador. Não há conjunto de treino para rotular.

O que veio de dados reais são os **parâmetros do simulador** — três constantes,
extraídas dos quatro logs da seção "O modelo físico", 203 s no total. Depois
disso o treino é sintético e ilimitado: 2 000 iterações × 8 192 passos =
**16,4 milhões de passos**, o equivalente a **91 horas de bancada**, em 79
minutos de CPU.

Os logs originais não ficam no diretório de trabalho; estão no histórico
(`git show a3984c8:logs/livre_01.csv`). Só precisam ser refeitos se a mecânica
mudar.

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

As seis baterias do `avaliar_sw.py`: sobe e fica · quanto do trilho usa
(segurança da bancada) · tremula · recupera de empurrão · aguenta mundo errado
· sobe de onde o carrinho parar.

### Hiperparâmetros, e a razão de cada um

| Parâmetro | Valor | Por que |
|---|---|---|
| rede | 2×64, tanh | 5 ou 6 entradas; mais capacidade só aumenta a variância |
| γ | **0,995** (0,99 no agente 1) | o swing-up leva ~175 passos e o prêmio vem no fim. Com 0,99 o horizonte efetivo é 100 passos e o início do bombeamento fica cego |
| λ (GAE) | 0,95 | compromisso usual viés/variância |
| ε (recorte) | 0,2 | valor do artigo original |
| entropia | **0,01** (0,004 no agente 1) | há uma armadilha (girar sem parar) e um ótimo distante: precisa explorar mais |
| coef. de valor | 0,5 | — |
| parada por KL | 0,02 | interrompe as épocas quando a política já mudou demais |
| recorte de gradiente | 0,5 | evita um lote atípico desestabilizar o treino |
| passos por lote | 8 192 | episódios longos (1 200 passos) |
| épocas / lote | 10 / 256 | — |
| taxa de aprendizado | 3e-4 | — |
| ação | gaussiana contínua, `logσ` aprendido e independente do estado | aceleração é contínua; discretizar perderia resolução |
| `--frac-topo` | 0,30 | 30% dos episódios começam no topo, para a habilidade de equilibrar não se perder enquanto ele aprende a subir |

O espaço de ação é contínuo, `a ∈ [−1, 1]` escalado para ±8 m/s². Três razões:
é exatamente a variável que o firmware já consome; permite comparação direta
com o PID; e produz movimento suave, com menos desgaste de correia que
chaveamento discreto.

### Observação e recompensa do agente 2

| Índice | Grandeza | Normalização |
|---|---|---|
| 0 | `x` | ÷ 0,24 m |
| 1 | `v` | ÷ 0,45 m/s |
| 2 | `sin(ψ)` | — |
| 3 | `cos(ψ)` | — |
| 4 | `−ω` | ÷ 15,0 rad/s |
| 5 | ação anterior | ÷ 8 m/s² |

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

A métrica de "melhor" também mudou. No agente 1 o episódio morria a 35°, então
*duração* media qualidade. Aqui só o trilho termina o episódio, e a duração é
quase sempre máxima: um agente que deixa a haste pendurada balançando dura o
mesmo que um que equilibra. A métrica passou a ser `frac_pe`, a fração da
**segunda metade** do episódio com |ψ| < 10° — responde a "subiu?" e "ficou?"
de uma vez.

### Randomização de domínio

A cada episódio: `L_eq` ±8%, os dois atritos de 0,5× a 1,8×, ganho de
aceleração ±10%, zero do encoder ±0,8°, atraso de 1 ou 2 passos. O agente nunca
treina duas vezes no mesmo mundo, e por isso não aprende a explorar
imperfeições de um modelo específico.

### Como obter melhores resultados

Sete itens, todos medidos neste projeto.

**1. A métrica de seleção satura — corrija isso primeiro.**
O critério é `if frac_pe > melhor`, estritamente maior. `frac_pe` chegou a
**100,0% na iteração 150 de 2 000**, e das 80 avaliações do treino **58
marcaram 100%** sem nenhuma ser salva. O `melhor.pt` embarcado é a iteração
150: ~73 dos 79 minutos de treino não estão nele.

E o treino continuou melhorando no que a métrica já não media:

| Avaliações | ψ em pé | ⟨\|x\|⟩ | Subida |
|---|---|---|---|
| it 1–250 | 0,664° | 43,6 mm | 2,82 s |
| it 251–500 | 0,627° | 40,2 mm | 2,91 s |
| it 501–1000 | 0,586° | 45,5 mm | 2,98 s |
| it 1001–1500 | 0,573° | 46,2 mm | 2,68 s |
| it 1501–2000 | **0,568°** | 52,0 mm | **2,51 s** |

O desempate natural é `psi_pe` (menor é melhor) quando `frac_pe` empata. Antes
de trocar, compare `agente.pt` com `melhor.pt` no `avaliar_sw.py`, olhando a
**bateria 2** (quanto do trilho usa): o `agente.pt` tem ψ 17% melhor e ⟨|x|⟩
43% pior, e a margem de trilho na subida é o fator apertado.

**2. Penalizar `Δa` penaliza a recuperação.** Resultado negativo medido: subir
o peso de `Δa` para suavizar a ação levou a recuperação de empurrões de 1,0
rad/s de 65% para 40%. Suavidade e recuperação são o mesmo recurso — a
capacidade de mudar de aceleração rápido. Não troque uma pela outra sem medir.

**3. Posição e ângulo são um compromisso explícito.** Uma tentativa de
reequilibrar os pesos levou ⟨|x|⟩ de 29 para 65 mm, porque em x = 65 mm a
punição de posição (0,041) ficou menor do que a preguiça economizava em `Δa`
(0,063). **Calcule os valores numéricos dos termos no ponto de operação
pretendido antes de treinar**, não depois.

**4. Verifique se a recompensa tem armadilha antes de gastar 79 minutos.**
Numa versão, "girar sem parar" pontuava 1,923 contra 2,000 do equilíbrio
perfeito — 96% do prêmio por um comportamento inútil. A correção foi
condicionar o bônus de precisão a `|ψ̇| < 2,0 rad/s`. Imprima o valor da
recompensa para três ou quatro comportamentos-limite (pendurado parado,
girando, em pé parado, em pé trepidando) e confira a ordem.

**5. Truncamento não é terminação** no bootstrap do crítico (seção "PPO: como
funciona").

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
| Agente 1 na bancada | 1,46° |
| **Agente 2 embarcado** | **0,75°** |

Meta mínima para um agente novo: sobreviver os 20 s em mais de 85% dos
episódios *com randomização de domínio ativa*, com ⟨|ψ|⟩ abaixo de 2° e ⟨|x|⟩
abaixo de 80 mm.

---

## Embarcar o modelo no ESP32

O modelo **não precisa ser reduzido**: 4 673 parâmetros são 18,3 KiB de
`.rodata` contra 4 MB de flash, e 4 544 MACs + 128 tanh contra 20 ms de
orçamento. Não entra TensorFlow Lite Micro nem ESP-DL — cujo intérprete sozinho
pesaria mais que a rede toda. Os pesos viram um array `const` e a inferência
são ~40 linhas de C.

| Recurso | Total | Pesos ocupam |
|---|---|---|
| Flash | 4 MB | 0,5% |
| SRAM interna | 520 KB | 3,6% |

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

O passo 3 é mais do que parece. O `politica_c.py` carrega o **mesmo** `.c` que
vai para a placa, via ctypes, e o `comparar_c.py` roda o simulador com ele — a
validação em malha fechada exercita o binário de produção, não uma
reimplementação. E o julgamento é contra um **piso de ruído medido** (o torch
com 1e-06 de tremor na saída), não contra um limiar fixo: num sistema que dobra
o erro a cada 105 ms, 1e-06 decorrelaciona a trajetória em ~2 s e `frac_pe`
muda por amostragem. O limiar fixo da primeira versão reprovou uma variante
cujo erro era puro arredondamento de float32.

**4. TESTE na placa, isolado.** Copie `politica.c`, `politica.h` e `oraculo.h`
para `firmware/bench_politica/`, grave, leia a 115200.

| Critério | Medido |
|---|---|
| `oraculo erro max < 1e-5` | 1,192e-06 |
| `p99 < 1000 µs` | 335 µs isolado, 418 µs com a ISR de 40 kHz |

Se o oráculo falhar, **pare** — nada mais do sketch significa coisa alguma.
Isolado de propósito: medido junto com o resto do firmware, o número não é
interpretável.

**5. Gravar `firmware/pendulo_rl/`** a 921600, e o teste a seco com `l`: a rede
calcula uma vez e nada se move. Com o carrinho no centro e a haste pendurada, o
valor esperado deste modelo é `bruta=-0.194 -> -1.554 m/s2`. **O sinal é o que
importa**: invertido significa `ENC_SINAL` ou entrada trocada na observação.

**6. TESTE observador — risco mecânico zero.** O PID atua, a rede só opina:

```bash
python conferir_local.py --porta /dev/cu.usbserial-0001 --seg 30 --csv conf.csv
```

Critério: `erro em bruta max < 1e-5`. Medido: 1,96e-06 em 1 494 amostras. É o
único teste que exercita a cadeia inteira — encoder → observação → rede → ação
— com sensores reais, e o último em que falhar é de graça.

**7. TESTE etapa 2 — o agente atua, com supervisor:**

```bash
python registrar_local.py --porta ... --etapa 2 --seg 60 --csv et2.csv
```

Levante a haste à mão. Critério: `intervencoes 0`, `em pe > 95%`,
`|x| max < 230 mm`. **Ignore ⟨\|x\|⟩ dos primeiros 6 s**: o PID entrega a haste
a até 10° e o agente faz uma excursão grande para agarrá-la. Olhe o regime —
medido: banda instantânea de 1,46 mm e ψ = −0,163° ± 0,084°.

**8. TESTE etapa 3 — agente completo, sem controle clássico:**

```bash
python registrar_local.py --porta ... --etapa 3 --seg 180 --csv et3.csv
```

Centralize antes. Haste pendurada e livre, **mão na chave de emergência** —
aqui o firmware para em vez de devolver ao PID. Critério: `flags 164` em 100%
das amostras, subiu, `em pe > 85%`, `|x| max < 230 mm`.

Com `--deixar-rodando` o script sai **sem** mandar parar: se o pêndulo continuar
equilibrando com o terminal fechado, está provado que a placa é autônoma.

### Decisões de implementação

- **Não use a LUT de tanh.** Os 128 `tanhf` são 70% dos 324 µs (425 ciclos
  cada, contra 5,1 por MAC), mas o gargalo inteiro cabe 60 vezes no orçamento.
  A LUT economizaria 215 µs de um orçamento com 19 600 sobrando, em troca de
  1000× menos precisão. Ela existe atrás de `POLITICA_TANH_LUT`, desligada.
- **Copie os pesos para SRAM no boot.** `const` vive em `.rodata`, atrás do
  cache de flash; um miss no meio da inferência é jitter, e jitter num sistema
  que dobra o erro a cada 105 ms não é detalhe.
- **Não use PSRAM** (no S3): é mais lenta que a SRAM interna e não resolve nada.
- **`#pragma GCC optimize ("O2")`** no `politica.c`: o core do ESP32 compila com
  `-Os` e a IDE não expõe a opção. Sem isso o número de latência sai pessimista.
- **Uma cópia só dos pesos.** `politica.c` e `politica.h` vivem em
  `firmware/pendulo_rl/`, a pasta gravada, e o teste e o `.so` compilam contra
  ela. Já houve divergência entre três cópias; agora não existe segunda cópia
  para divergir. `teste_politica.c` não pode ficar num sketch: tem `main()` e
  colidiria com o do core.
- **A observação é montada na placa a partir dos valores transmitidos**, lidos
  num instantâneo único por fronteira de 20 ms. Sem isso a ISR pode alterar
  `posicao` entre a leitura do pacote e a da observação, e a comparação acusa
  diferença onde não há bug. Sai de graça porque o pacote carrega `v` e `omega`
  como float32 e `pos` e `enc` como int32: não há quantização no caminho.
- **O atraso de um passo é preservado de graça**: a rede escreve em
  `aRL_pendente` e a máquina existente só promove na fronteira seguinte. É o
  mesmo atraso que o simulador modelou.
- **`limiteB / 2.0f`, não `/ 2`.** `limiteB` é `long`, e com curso ímpar a
  divisão inteira erra meio passo. Fisicamente é nada, mas a ponte sempre usou
  10 701,5 e foi com esse `x` que o agente passou a validação — a rede tem de
  ver o número que a validou.
- **A ISR do gerador de passo é o custo oculto.** Com ela a 40 kHz, a
  inferência vai de 335 para 418 µs de p99 — ~4,9 µs por interrupção, ou ~20%
  de um núcleo. A causa provável é `digitalWrite`/`digitalRead` não estarem em
  IRAM no core do Arduino: trocar por acesso direto a registrador
  (`GPIO.out_w1ts`/`out_w1tc`) é a otimização mais barata que resta.

---

## Operar o agente embarcado

1. Gravar `pend_rl/firmware/pendulo_rl/` (ESP32 Dev Module, **240 MHz**).
2. Monitor serial a **921600**. Confirmar no boot:
   `# Rede embarcada: 6->64->64->1, 4673 parametros, 18692 bytes em SRAM`
3. Com a haste **pendurada e parada**: `z`, depois `c`.
4. `l` — liga a rede local e imprime o teste a seco. Nada se move.
5. `g` — agente completo. Ele ergue a haste e a mantém.

Para parar: `s` **não funciona** em modo binário, porque o firmware só lê o
pacote de 7 bytes. Aperte **EN** na placa ou a chave de emergência.

Depois de um reset os três alternadores voltam a desligado: `z` `c` `l` `g`.

### Teclas do firmware de RL (921600)

`z` zera o encoder (haste pendurada e parada) · `c` centraliza ·
`l` rede local, roda na placa · `o` observador, o PID atua e a rede só opina ·
`g` agente completo · `r` entrega após captura · `s` para ·
`m` supervisor PID · `v` limitador de velocidade · `d` telemetria detalhada

### Com registro em CSV

```bash
source venv/bin/activate && cd pend_rl
python registrar_local.py --porta /dev/cu.usbserial-0001 --etapa 3 --seg 180 --csv et3.csv
```

O script leva os alternadores a um estado conhecido (eles são *toggles*: depois
de uma conferência, `rlObserva` fica ligado e a etapa 3 rodaria com o PID
atuando, parecendo sucesso do agente), entra no modo e grava.

### Protocolo serial

Serial a 921 600 baud, binário, com CRC-8/SMBUS nos dois sentidos: **21 bytes
de subida** (posição em passos, contagem do encoder, velocidade, velocidade
angular, flags, sequência) e **7 bytes de descida** (ação em int16, comando,
sequência). O número de sequência detecta pacote perdido; os flags informam o
modo do firmware (`F_RL` 0x04, `F_SUPERV` 0x08, `F_PERDEU` 0x10,
`F_ZERO_OK` 0x20, `F_OBSERVA` 0x40, `F_COMPLETO` 0x80).

Texto e binário compartilham a porta, mas não o estado: **em modo RL o firmware
não lê texto algum**. A rede embarcada publica a ação pretendida como linha de
texto `NET <seq> <bruta>`, amarrada à amostra pelo `seq` — o pacote binário fica
intacto e nenhum parser de log antigo quebra.

Com a rede embarcada o PC saiu do laço. O orçamento de tempo que importa hoje é
o da placa (335 µs de inferência em 20 ms). Na arquitetura anterior, com a
política no PC, era: inferência 0,04 ms, ida e volta pela serial 0,05 ms médio,
intervalo entre ações 20,1 ms médio e 21,5 ms no p99, watchdog de 100 ms. A
folga parecia enorme e não era — o USB-serial do macOS entrega pacotes em lote,
e lacunas de 45 a 53 ms foram medidas. Foi por isso que o watchdog subiu de 60
para 100 ms.

---

## Como clássico e agente trabalham juntos

Os dois **nunca atuam ao mesmo tempo.** Não há mistura de sinais nem média
ponderada: o controle é posse exclusiva, com condições definidas para trocar de
mãos.

### Entrega: o clássico passa o bastão

O comando `r` não entrega na hora — arma uma intenção. A troca só acontece
quando os três critérios valem ao mesmo tempo:

```c
if (rlAposCaptura &&
    fabsf(psi)    < 3.0f * PI / 180.0f &&   // |psi| < 3 graus
    fabsf(psiDot) < 0.5f &&                 // |psiDot| < 0,5 rad/s
    fabsf(x)      < X_ENTREGA_RL) {         // |x| < 120 mm
```

Enquanto faltar algum, o firmware diz qual. Entregar um pêndulo já caindo seria
pôr o agente num estado que ele mal viu no treino.

### Supervisão: o clássico vigia e pode retomar

Com o agente no comando, o PID continua calculando em silêncio a 1 kHz, sem
atuar. Ele retoma em três situações:

| Gatilho | Condição | Motivo registrado |
|---|---|---|
| Ângulo | \|ψ\| > 20° | `angulo acima do limite do agente` |
| Posição | \|x\| > 240 mm | `carrinho fora da faixa util` |
| Comunicação | 100 ms sem ação do PC | `comunicacao perdida` |

A retomada é abrupta e completa: `modo = EQUILIBRIO`, protocolo binário
desligado, integral zerada. Os 20° de gatilho contra os 35° de limite do
firmware dão ao PID **15° de margem** — e ele precisa, porque o erro dobra a
cada 105 ms.

Nas nove etapas de validação do agente 1 esse supervisor **nunca disparou**.

### Observação: ambos decidem, só o clássico atua

O comando `o` cria um terceiro estado. O firmware entra em modo RL, publica o
estado a 50 Hz, recebe (ou calcula) a ação da rede — e a descarta. Quem move o
carrinho é o PID.

Serve para comparar as duas decisões no mesmo estado sem risco nenhum. Foi
assim que a correlação de +0,916 foi medida com o agente 1, e é assim que a
conferência bit a bit da rede embarcada é feita hoje.

### No agente completo, a cooperação acaba

Com o comando `g` o arranjo muda de natureza. O supervisor de ângulo é
**desligado**, porque durante o swing-up |ψ| passa por 180° e ele dispararia no
primeiro ciclo. Não há entrega: o agente recebe a haste pendurada e parada.

O que resta não é controle clássico, é **limite físico** — saturação de
velocidade e aceleração, freio de parede e fins de curso em interrupção. Nenhum
deles calcula uma ação; apenas impedem que a do agente ultrapasse o que o
hardware aguenta.

A saída por falha também muda de destino: sem supervisor, bater no trilho leva
a `modo = PARADO`, não ao PID. Entregar uma haste possivelmente pendurada a um
PID de equilíbrio seria pior que parar.

### Camadas de proteção

| Camada | Onde | O que faz |
|---|---|---|
| Supervisor PID | firmware | \|ψ\| > 20° → o PID retoma (tem até 35° para salvar) |
| Sanidade da rede | firmware | saída não finita ou \|bruta\| > 100 → `PARADO` |
| Watchdog | firmware | 100 ms sem ação → devolve ao PID ou para (inativo com a rede local) |
| Saturações e freio de parede | firmware | velocidade, aceleração e curso |
| Aborto pelo PC | ponte | \|ψ\| > 18° ou \|x\| > 235 mm (só com a política no PC) |
| Fins de curso | ISR | cortam os pulsos na interrupção |
| Chave de emergência | o operador | em série com os 24 V |

As cinco primeiras são software e podem ter bugs — tiveram. As duas últimas, não.

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
| `pend_rl/modelos_sw/melhor.pt` | **o modelo embarcado** (iteração 150) |
| `pend_rl/modelos_sw/agente.pt` | pesos da última iteração (ver "melhores resultados") |
| `pend_rl/modelos_sw/historico.csv` | uma linha por iteração do treino |
| `pend_rl/requirements.txt` | numpy, scipy, pandas, matplotlib, torch, pyserial |

### Firmware

| Pasta | O que é |
|---|---|
| `firmware/pendulo_rl/` | **o que está gravado**: `.ino` + `politica.c`/`.h` + `estado_rl.h`. Única cópia dos pesos |
| `firmware/politica/` | `oraculo.h` + `teste_politica.c`, o teste em C |
| `firmware/bench_politica/` | benchmark de latência isolado. O cabeçalho do `.ino` diz quais arquivos trazer |

`estado_rl.h` existe porque o pré-processador do Arduino injeta os protótipos
das funções no **topo** do sketch, acima de qualquer `struct` definida ali —
o que dá `error: 'EstadoRL' does not name a type`. Definido num header incluído,
o tipo passa a ser conhecido antes dos protótipos.

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
| as nove etapas com os critérios originais | `git show 1bfbf23:pend_rl/roteiro_bancada.md` |
| a identificação em detalhe | `git show a2b1940~1:pend_rl/identificacao.md` |
