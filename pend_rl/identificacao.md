# Identificação do modelo e decisões de projeto

Documento de referência do simulador `pendulo_env.py`.
Todos os números vêm dos logs reais gravados em 20/10/2026.

---

## 1. Os dados de partida

| Arquivo | Duração | Conteúdo | Para que serviu |
|---|---|---|---|
| `livre_01.csv` | 72,4 s | Oscilação livre, carrinho parado (modo 0) | Comprimento equivalente e atrito |
| `pid_01.csv` | 41,3 s | PID equilibrando (modo 4), 37,9 s úteis | Conferência da lei de controle, ruído, atraso |
| `swing_01.csv` | 51,0 s | Swing-up + equilíbrio | Validação do acoplamento carrinho→haste |
| `swing_02.csv` | 38,6 s | Idem | Validação cruzada |

**Qualidade:** dt medido de 10,04 ms com desvio de 0,049 ms, e **zero amostras perdidas**
nos quatro arquivos. O registrador não interferiu no controle.

---

## 2. Parâmetros identificados

### 2.1 Comprimento equivalente — pelo período

O período de um pêndulo depende da amplitude. Medi 64 cruzamentos por zero em
`livre_01.csv` e extraí o período de cada ciclo, junto com a amplitude daquele ciclo.
Depois corrigi cada período pela fórmula exata (integral elíptica) para extrapolar
à amplitude zero:

```
T(A) = T0 · (2/π) · K(sin²(A/2))
```

| Grandeza | Valor |
|---|---|
| Pares período/amplitude usados | 55 (de 54,8° a 3,8°) |
| Período bruto: primeiro → último | 1,0086 s → 0,9536 s |
| **T0 (amplitude → 0)** | **0,9508 s** (desvio de apenas 0,8 ms entre os 55 ciclos) |
| **ω₀** | **6,6085 rad/s** |
| **L_eq** | **224,63 mm** |
| L_eq que estava no firmware | 220,0 mm → **erro de +2,1%** |

**Conferência física independente.** A haste mede 255 mm do eixo à ponta e pesa 42 g,
com a maior massa na extremidade. Para uma haste uniforme, L_eq seria 2ℓ/3 = 170 mm;
com toda a massa na ponta, 255 mm. O valor medido, 224,63 mm, corresponde a:

```
L_eq/ℓ = (m_haste/3 + m_ponta) / (m_haste/2 + m_ponta) = 0,881
→ m_ponta ≈ 20 g   e   m_haste ≈ 22 g
```

Ou seja, cerca de **47% da massa concentrada na extremidade** — exatamente a descrição
física da peça. Dois caminhos totalmente distintos (período de oscilação × geometria e
massa) chegam ao mesmo número.

> **A massa de 42 g não entra na dinâmica.** O motor de passo impõe a trajetória do
> carrinho independentemente da reação do pêndulo — por isso a equação é mais simples
> que a do CartPole clássico. A massa só importaria para conferir torque: 42 g pedem
> ~0,4 N contra os ~39 N que o NEMA 17 entrega pela polia. Folga de 100×.

> **Decisão:** usar `L_EQ = 0.22463`. O desvio de 0,8 ms entre 55 ciclos independentes
> mostra que a correção elíptica está certa — se o modelo de período estivesse errado,
> os T0 estimados divergiriam conforme a amplitude cai.

### 2.2 Atrito — pelo envelope de decaimento

A forma do decaimento diz qual é o tipo de atrito:

| Ajuste do envelope | R² | Interpretação |
|---|---|---|
| Exponencial (viscoso puro) | 0,928 | — |
| **Linear (Coulomb/seco)** | **0,960** | mancal com atrito seco dominante |

Como nenhum dos dois é perfeito, ajustei **os dois juntos** pela taxa de queda da amplitude:

```
dA/dt = −(b/2)·A − 2c/(π·ω₀)
```

| Coeficiente | Valor | Significado |
|---|---|---|
| **b** (viscoso) | **0,06274 1/s** | proporcional a ω |
| **c** (Coulomb) | **0,04984 rad/s²** | constante, opõe-se ao movimento |

> **Decisão:** manter os dois termos. O Coulomb é implementado como
> `c · tanh(ω/0,02)` em vez de `c · sign(ω)`, porque `sign` é descontínuo e
> gera trepidação numérica quando a haste está quase parada.

### 2.3 Trilho e atuador — dos arquivos de constantes

| Parâmetro | Valor | Origem |
|---|---|---|
| Passos por metro | 40 000 | **teórico** (ver pendência na seção 6) |
| Curso útil | 0,5329 m | medido pelo homing (21 316 passos) |
| Limite físico de x | ±0,2665 m | metade do curso |
| Velocidade máxima | 0,45 m/s | 18 000 passos/s ÷ 40 000 |
| Aceleração máxima | 8,0 m/s² | `A_BAL_MAX` |
| Passo do encoder | 0,15° | confirmado: 100% dos saltos são múltiplos inteiros |

---

## 3. As três validações

### 3.1 Oscilação livre (haste sozinha)

Simulação integrada a 1 kHz, comparada ao log em janelas de 5 s iniciadas em
cruzamentos por zero (onde a velocidade é máxima e a estimativa inicial é confiável):

| Amplitude da janela | Erro RMS |
|---|---|
| 55,5° | 0,85° |
| 46,9° | 0,26° |
| 39,1° | 0,66° |
| 26,7° | 0,19° |
| 17,8° | 0,10° |
| 10,9° | 0,11° |
| 8,3° | 0,41° |
| **Média** | **0,37°** |

> **Armadilha encontrada:** iniciar a janela num ponto de retorno dá erro de 45°.
> Não é falha do modelo: ali a haste quase não se move, a quantização de 0,15°
> domina a estimativa de velocidade, e a fase inicial sai errada. Sempre inicialize
> uma simulação de pêndulo onde o sinal está mudando rápido.

### 3.2 Swing-up (com o carrinho acelerando)

Este teste valida o termo de acoplamento `−a·cos(φ)/L`, que a oscilação livre não exercita.

> **Armadilha mais importante de todas:** o `a_ms2` do log é a aceleração **pedida**
> pelo controlador. O firmware depois satura a velocidade em ±0,45 m/s e aplica o
> freio de parede. Durante o swing-up a velocidade fica saturada 19–25% do tempo:
> nesses instantes o log diz "a = 8 m/s²" enquanto o carrinho, já no teto,
> tem aceleração **zero**.
>
> A aceleração verdadeira é `dv/dt`, derivada da coluna `v_ms`.
> Com `a_ms2`: erro de 44 a 105°. Com `dv/dt`: erro de 0,6 a 3,3°.

| Janela (1,2 s) | Excursão | Erro RMS |
|---|---|---|
| swing_01, t=0,3 s | 118° | 0,95° |
| swing_01, t=1,0 s | 190° | 0,57° |
| swing_01, t=1,8 s | 237° | 1,29° |
| swing_02, t=0,3 s | 118° | 0,94° |
| swing_02, t=1,0 s | 193° | 1,60° |

### 3.3 Perto do topo — o horizonte de previsão

O pêndulo invertido é instável com constante de tempo τ = √(L/g) = 0,151 s.
Qualquer erro **dobra a cada 105 ms**. Medido em 30 janelas do trecho de equilíbrio:

| Horizonte | Erro RMS | Fator vs. anterior |
|---|---|---|
| 0,02 s (passo do agente) | ~0,06° | — |
| 0,1 s | 0,30° | — |
| 0,2 s | 0,70° | 2,3× |
| 0,3 s | 1,28° | 1,8× |
| 0,5 s | 3,90° | 3,0× |
| 1,0 s | 54,2° | 14× |

> Isso **confirma** o modelo em vez de contestá-lo: a taxa de divergência medida
> bate com a instabilidade teórica. Validar um pêndulo invertido por simulação em
> malha aberta por mais de ~0,3 s é matematicamente impossível, não importa quão
> bom seja o modelo.

### 3.4 Conferência da lei de controle

Reconstruí `a = 40·ψ + 5,5·ψ̇ + 1,5·x + 17,5·v` a partir das colunas do log:

- resíduo médio: −0,174 m/s², desvio 0,060 m/s²
- **100% das amostras com resíduo dentro de ±0,5**, que é exatamente o limite `I_MAX`

O resíduo *é* o termo integral. Isso prova que a interpretação das colunas do CSV
e da lei de controle está exata.

---

## 4. A descoberta que muda o projeto: taxa de controle

Rodando o PID **com os ganhos exatos do firmware** dentro do simulador:

| Taxa | Sobrevive 20 s | ⟨\|ψ\|⟩ |
|---|---|---|
| 1000 Hz | 20/20 | 0,29° |
| 500 Hz | 20/20 | 0,30° |
| 200 Hz | 20/20 | 0,38° |
| 100 Hz | 20/20 | 4,56° |
| 50 Hz | 20/20 | 5,70° |
| 25 Hz | 0/20 | falha |

Parecia que 50 Hz seria limitante demais para o PPO. **Não é.** Busquei os melhores
ganhos *para 50 Hz*:

| Conjunto | KP | KD | KX | KV | ⟨\|ψ\|⟩ a 50 Hz |
|---|---|---|---|---|---|
| Original (sintonizado a 1 kHz) | 40 | 5,5 | 1,5 | 17,5 | 5,68° |
| **Reajustado para 50 Hz** | 41,1 | 4,51 | 0,44 | **1,3** | **0,24°** |

O culpado é o `KV`. A 1 kHz, `KV·dt = 0,0175`. A 50 Hz, `KV·dt = 0,35` — realimentação
de velocidade grande demais por passo, que desestabiliza o laço discreto.

> **Decisão:** manter o agente a **50 Hz** com a política no PC. A arquitetura simples
> (serial USB, ping-pong de 20 ms) está liberada. O PPO vai descobrir sozinho ganhos
> adequados ao seu próprio dt — essa é justamente uma vantagem dele sobre transplantar
> ganhos de outro regime.

---

## 5. A perturbação que realmente importa

Testando cada fonte de incerteza isoladamente, com os bons ganhos de 50 Hz:

| Perturbação | Faixa | Sobrevive |
|---|---|---|
| Comprimento L | ±8% | 100% |
| Atrito b, c | ×0,5 a ×1,8 | 100% |
| Ganho de aceleração | ±10% | 100% |
| Escala de posição | ±5% | 100% |
| Atraso de atuação | 1 ou 2 passos | 95% |
| **Offset de zero do ângulo** | **σ = 0,8°** | **10%** |

O offset de zero é devastador, e pelo motivo certo: se o controlador acredita que a
vertical está 0,8° deslocada, ele segura a haste nesse ângulo errado. Isso exige
aceleração constante num sentido, e o carrinho corre até a parede.

**Isso aconteceu na sua bancada.** Em `pid_01.csv`, `x` tem média de +175 mm e chega
a 256 mm — o carrinho derivou até o limite. Não é defeito do PID: é a calibração do zero.

O antídoto é realimentação de posição mais forte:

| KX | Sobrevive com offset |
|---|---|
| 0,44 | 10% |
| 1,5 | 25% |
| 5 | 65% |
| **8** | **85%** |
| 12 | 30% |
| 18 | 0% |

> **Decisão:** manter o offset na randomização de domínio (σ = 0,8°) e incluir o
> termo de posição na recompensa. O PPO precisa aprender a tolerar zero mal calibrado,
> porque o hardware real sempre terá um. E como política não-linear, ele pode fazer
> melhor que os 85% do ganho fixo ótimo.

---

## 6. Pendência em aberto: a escala do trilho

Três medidas discordam:

| Fonte | passos/m | Curso implicado |
|---|---|---|
| Teórico (GT2 20 dentes, 1600 passos/volta) | 40 000 | 533 mm |
| **Dinâmica do swing-up** (ver abaixo) | **39 200 ± 1200** | 548 mm |
| Régua: 600 mm entre fins de curso | 35 800 | 600 mm |

### A medida pela dinâmica

Multipliquei a aceleração do log por um fator `k` e procurei o `k` que minimiza o erro
de validação do swing-up. Como o acoplamento haste-carrinho é `−a·cos(φ)/L`, e `L` veio
da oscilação livre (que não envolve o carrinho), esta é uma medida **independente**:

| k | passos/m | Erro RMS |
|---|---|---|
| 0,96 | 41 667 | 2,15° |
| **1,00** | **40 000** | **1,22°** |
| **1,02** | **39 216** | **1,22°** ← mínimo |
| 1,04 | 38 462 | 1,64° |
| 1,10 | 36 364 | 3,63° |
| **1,12** | **35 714** | **4,37°** ← a escala da régua |

A escala da régua dá erro 3,6× maior. A física não a aceita.

> **Subproduto útil:** k ligeiramente acima de 1 significa que **não há perda de passos**
> durante o swing-up, mesmo com aceleração de 8 m/s². Se houvesse, o carrinho andaria
> menos que o comandado e o melhor k ficaria abaixo de 1.

### Como resolver de forma definitiva

A divergência provavelmente está no que foi medido: "antes de encostar nas chaves"
não é o mesmo que "onde as chaves acionam" — uma microchave de alavanca aciona depois
de um curso do êmbolo, e dois lados somam.

Medida inequívoca, usando só os comandos que já existem no firmware:

1. Envie `s`, depois `a` — o carrinho vai à posição 0 (80 passos após a chave A).
2. **Marque a lateral do carrinho** com um lápis, na mesa ou no perfil.
3. Envie `b` — o carrinho vai a `limiteB` (80 passos antes da chave B).
4. Marque de novo e **meça entre as duas marcas**.

Essa distância corresponde a exatamente `limiteB = 21 316 passos`. Então:

```
passosPorM = 21316 / (distância medida em metros)
```

| Se medir | passos/m |
|---|---|
| 533 mm | 40 000 (teórico confirmado) |
| 544 mm | 39 200 (confirma a dinâmica) |
| 595 mm | 35 800 (confirma a régua anterior) |

Depois preencha `CURSO_REAL_M` no firmware com a distância **entre os pontos de
acionamento**, que é essa medida mais 4 mm (os dois recuos de 80 passos).

Até lá, o simulador usa 40 000 passos/m e randomiza a escala em **±8%** para cobrir
toda a faixa em disputa.

---

## 7. Decisões do ambiente de RL

### Espaço de observação (5 valores, normalizados)

| Índice | Grandeza | Normalização | Por quê |
|---|---|---|---|
| 0 | x | ÷ 0,24 m | limite de software |
| 1 | v | ÷ 0,45 m/s | saturação do atuador |
| 2 | ψ | ÷ 35° | limite do episódio |
| 3 | ψ̇ | ÷ 6,0 rad/s | valor empírico dos logs |
| 4 | ação anterior | ÷ 8 m/s² | **compensa o atraso de atuação** |

A quinta entrada não existe no CartPole padrão. Ela entra porque o atraso de um passo
viola a hipótese de Markov: o estado atual não descreve o que já está a caminho.
Dando a última ação ao agente, a informação volta a ser completa.

### Espaço de ação

Contínuo, `a ∈ [−1, 1]` escalado para ±8 m/s². Razões:

1. É exatamente a variável que o firmware já consome (`a` do PID).
2. Permite comparação direta com o PID e, no futuro, partir dele.
3. Produz movimento suave — menos desgaste de correia que chaveamento discreto.

### Recompensa

```
r = 1,0
  − 0,60·(|ψ|/ψmax)²      fique vertical
  − 0,25·(|x|/xmax)²      fique perto do centro   ← combate o drift da seção 5
  − 0,05·|Δa|/amax        não trepide
r = −50  ao cair ou sair do trilho
```

### Término

| Condição | Valor | Origem |
|---|---|---|
| \|ψ\| > 35° | ψ_LIM | `SAIDA_GRAUS` do firmware |
| \|x\| > 0,24 m | X_LIM | dentro do limite físico de 0,2665 |
| 1000 passos | 20 s | sucesso (truncamento, sem punição) |

### O que o simulador replica do hardware

- integração interna a 1 kHz, com decisão a 50 Hz (igual ao firmware)
- quantização do encoder em 0,15°
- o mesmo filtro de ω: janela de 8 amostras + EMA de 50%
- saturação de velocidade e freio de parede, com as mesmas constantes
- atraso de atuação de 1 passo (configurável)

---

## 8. Referência de desempenho a bater

| Fonte | ⟨\|ψ\|⟩ | Observação |
|---|---|---|
| PID na bancada real | 1,90° | derivou até a parede (x̄ = 175 mm) |
| PID original no simulador a 50 Hz | 5,68° | ganhos de 1 kHz não transferem |
| Melhor PID possível a 50 Hz | 0,24° | sem offset de zero |
| Melhor PID a 50 Hz com offset | — | 85% de sobrevivência |

**Meta para o PPO:** sobreviver os 20 s em mais de 85% dos episódios *com randomização
de domínio ativa*, mantendo ⟨|ψ|⟩ abaixo de 2° e ⟨|x|⟩ abaixo de 80 mm.
