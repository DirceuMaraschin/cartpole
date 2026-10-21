# O PPO deste projeto, parte por parte

Acompanha `ppo.py`, `treinar.py` e `avaliar.py`.
Cada seção explica **o que a peça faz**, **por que ela existe** e **qual decisão foi tomada aqui**.

---

## 1. O que o agente vê e o que ele faz

### Observação: 5 números

| # | Grandeza | Normalizado por | Por quê |
|---|---|---|---|
| 0 | x (posição) | 0,24 m | limite de software |
| 1 | v (velocidade) | 0,45 m/s | saturação do motor |
| 2 | ψ (inclinação) | 35° | limite do episódio |
| 3 | ψ̇ (vel. angular) | 6,0 rad/s | faixa vista nos logs |
| 4 | **ação anterior** | 8 m/s² | compensa o atraso |

As quatro primeiras são o estado físico. A quinta merece explicação.

O ambiente tem **atraso de um passo**: a ação decidida agora só atua daqui a 20 ms.
Isso quebra a hipótese de Markov — o estado atual não conta o que já está a caminho.
É como dirigir vendo a estrada com meio segundo de atraso: você precisa lembrar o que
acabou de fazer com o volante. Dando a última ação ao agente, a informação volta a ser
completa.

**Normalizar não é detalhe.** Sem isso, x vale ~0,02 e ψ̇ vale ~3 — duas ordens de
grandeza de diferença. A rede gastaria boa parte do treinamento só descobrindo a escala
de cada entrada.

### Ação: um número contínuo

`a ∈ [−1, 1]` que vira aceleração em ±8 m/s².

**Por que contínua e não discreta.** Três razões:

1. É exatamente a variável que o firmware já consome (o `a` do PID). Zero adaptação.
2. Permite comparar diretamente com o PID, no mesmo eixo.
3. Produz movimento suave. Ação discreta obrigaria o agente a chavear entre valores
   fixos, e o carrinho trepidaria — desgaste de correia e vibração no hardware real.

O custo é que a política precisa ser uma **gaussiana** em vez de uma distribuição
categórica, o que exige um parâmetro a mais (o desvio-padrão). Vale a pena.

---

## 2. As duas redes

```
estado (5) ──> ATOR    (64 → 64 → 1)  ──> média da ação
           └─> CRÍTICO (64 → 64 → 1)  ──> valor do estado
```

### Ator: a política

Responde "o que eu faço agora?". A saída é a **média** de uma gaussiana; o desvio vem
de um parâmetro separado. Durante a coleta a ação é **sorteada** dessa gaussiana
(exploração); na execução final usamos a média (sem tremor).

### Crítico: a função de valor

Responde "quão boa é esta situação?" — um único número, a recompensa total que ele
espera acumular daqui para frente.

**Por que o Crítico é indispensável.** Sem ele, só daria para julgar uma ação esperando
o episódio inteiro acabar. Uma ação boa no passo 10 receberia crédito pelo que
aconteceu no passo 400. O Crítico fornece uma **linha de base**: se o retorno foi 190 e
ele previa 180, a ação rendeu +10 — foi boa. Se previa 200, a mesma nota 190 significa
que foi **ruim**. Essa diferença é a Vantagem, e é o sinal que treina o Ator.

### Decisões de arquitetura

| Decisão | Valor | Motivo |
|---|---|---|
| Tamanho | 2 camadas de 64 | 5 entradas, 1 saída. Rede maior não ajuda e a inferência roda dentro do laço de 20 ms |
| Ativação | **tanh**, não ReLU | tanh satura suavemente em [−1,1]. No hardware o estado sai da faixa vista no treino, e ReLU produziria saídas explosivas |
| Inicialização | ortogonal | mantém a escala do sinal estável entre camadas; é o padrão nas implementações de referência do PPO |
| Ganho da última camada do Ator | **0,01** | a política começa quase em zero, ou seja, parada. Começar agitado destrói o pêndulo antes de qualquer aprendizado |
| Redes separadas | sim | o Ator precisa de precisão perto de zero; o Crítico precisa prever números de até ~900. Compartilhar camadas faz um gradiente atrapalhar o outro |

### O desvio-padrão da política

É um **parâmetro treinável**, não depende do estado. Começa em `exp(−0,5) = 0,61`
(bastante exploração) e o próprio agente o reduz conforme ganha confiança. No
treinamento deste projeto ele caiu de 0,61 para ~0,25 — e observar essa queda é um dos
melhores diagnósticos de que o aprendizado está indo bem.

---

## 3. GAE: medindo o quanto cada ação foi boa

### O erro de diferença temporal

```
δ_t = r_t + γ·V(s_{t+1}) − V(s_t)
```

Em português: *"o que eu realmente ganhei agora, mais o quanto vale o lugar onde parei,
menos o que eu esperava valer antes de agir"*. É a **surpresa**. Positiva, a ação
superou a expectativa; negativa, decepcionou.

### Somando as surpresas futuras

```
Â_t = δ_t + (γλ)·δ_{t+1} + (γλ)²·δ_{t+2} + ...
```

Usar só `δ_t` é rápido mas **enviesado** (depende inteiramente do Crítico, que no começo
chuta). Usar o episódio inteiro elimina o viés mas traz **variância altíssima**. O λ faz
a ponte.

| Parâmetro | Valor | Efeito |
|---|---|---|
| **γ = 0,99** | horizonte de ~100 passos | A 50 Hz, isso é "enxergar" 2 segundos à frente. Suficiente para antecipar uma queda ou a parede, sem o aprendizado ficar lento |
| **λ = 0,95** | equilíbrio padrão | No hardware o sensor já tem ruído; λ perto de 1 tornaria o treinamento instável |

### A armadilha do truncamento

Um episódio pode terminar de duas maneiras:

- **Caiu** (ψ > 35° ou bateu no limite): o futuro realmente acabou. `V(s_final) = 0`.
- **Truncou** (20 s completos): o futuro **existe**, só paramos de olhar.

Se tratarmos os dois iguais, o agente aprende que sobreviver até o fim é ruim, porque o
retorno some de repente. A solução é usar `V(s_final)` como estimativa do que viria —
chama-se *bootstrap*. É um erro comum e silencioso: o agente simplesmente nunca melhora
e ninguém entende por quê.

---

## 4. O clipping: o coração do PPO

### O problema

Coletar dados é caro. Queremos reusar o mesmo lote várias vezes. Mas os dados foram
gerados pela política **antiga**, e estamos avaliando a **nova**. A correção é a razão:

```
r_t(θ) = π_nova(a|s) / π_antiga(a|s)
```

r = 1,3 significa "esta ação ficou 30% mais provável depois que atualizei a rede".

Aí mora o perigo: se a vantagem é positiva, o otimizador percebe que basta aumentar `r`
indefinidamente para aumentar o objetivo. Ele empurraria a probabilidade daquela ação
para 100% com base em um punhado de amostras — e a política despencaria.

### A solução

```
L = E[ min( r·Â ,  clip(r, 1−ε, 1+ε)·Â ) ]       com ε = 0,2
```

O `min` torna o objetivo **pessimista**: o PPO sempre adota a estimativa mais
conservadora. Onde a curva fica horizontal, a derivada é zero e o otimizador perde o
incentivo de continuar empurrando naquela direção.

> **Analogia.** Um técnico corrigindo um atleta. Acertou? Elogia — mas até certo ponto.
> Depois disso ele para: *"já entendi que você gosta dessa jogada, não vamos exagerar
> com base em um jogo só"*. Errou? Corrige — sem destruir a confiança.

**Assimetria proposital:** se a política se moveu na direção errada e tornou *muito*
mais provável uma ação comprovadamente ruim, isso **não** é recortado. O clipping
protege contra otimismo excessivo, nunca contra a correção de um engano.

---

## 5. A perda total

```
L = L_clip  +  0,5·L_crítico  −  0,004·entropia
```

| Termo | Coef. | Função |
|---|---|---|
| L_clip | — | otimiza as ações |
| L_crítico | **c₁ = 0,5** | erro quadrático de V(s) contra o retorno observado |
| entropia | **c₂ = 0,004** | paga o agente para continuar explorando |

**Sobre a entropia.** Sem esse termo é comum o agente "travar" cedo numa estratégia
medíocre. Com ele demais, nunca converge. Usei 0,004 (abaixo do 0,01 usual) porque a
ação é contínua e a gaussiana já explora por conta própria — o desvio-padrão treinável
faz boa parte desse trabalho.

---

## 6. Proteções do passo de otimização

| Proteção | Valor | O que evita |
|---|---|---|
| Normalizar as vantagens | média 0, desvio 1 | sem isso, mexer na recompensa exigiria re-sintonizar a taxa de aprendizado |
| Recorte da norma do gradiente | 0,5 | atualizações explosivas, independente do clipping da razão |
| **Parada antecipada por KL** | alvo 0,02 | se a política já se afastou demais, as épocas restantes só extrapolam |
| Taxa de aprendizado | 3×10⁻⁴ | padrão do PPO; funcionou de primeira |
| Épocas por lote | 10 | aproveita bem cada lote sem saturar o clipping |
| Minilote | 64 | gradientes estáveis sem deixar a atualização lenta |
| Passos por lote | 4096 | ~82 s de bancada simulada por iteração |

---

## 7. A recompensa, e por que ela tem quatro termos

```
r = 1,0
  − 0,60·(|ψ|/35°)²        fique vertical
  − 0,25·(|x|/0,24)²       fique perto do centro
  − 0,05·|Δa|/8            não trepide
r = −50                     ao cair
```

| Termo | O que ensina | Consequência de tirar |
|---|---|---|
| +1 por passo | sobreviva | sem isso não há objetivo |
| −0,60·ψ² | não basta não cair, fique **reto** | o agente oscila a 20° e ainda "ganha" |
| −0,25·x² | volte ao centro | **o carrinho encosta na parede** — foi o que aconteceu na sua bancada |
| −0,05·|Δa| | não chaveie à toa | correia e motor sofrem; vibração mecânica |
| −50 ao cair | caiu é ruim | sem punição, cair cedo pode render mais que lutar |

Os termos quadráticos punem pouco perto de zero e muito perto do limite — exatamente o
comportamento desejado.

> **Regra de ouro.** Cada peso novo é uma oportunidade de o agente achar uma brecha.
> Comece com o mínimo que funcione, observe o comportamento e acrescente um termo de
> cada vez. E se mudar a recompensa, **todo o aprendizado anterior precisa ser refeito**.

---

## 8. A lição mais cara deste projeto

O primeiro agente treinado deu **100% de sucesso, 0,70° de oscilação, 21 mm do centro**
e suavidade de 0,30. Números melhores que o PID em tudo.

E era inútil.

A bateria de testes revelou: ele recuperava de 6° mas falhava em 10°, e **não sobrevivia
a nenhum empurrão** — enquanto o PID aguentava 55% deles. A causa estava numa linha do
`reset()`: durante o treinamento o ângulo inicial era sorteado em ±3° e o carrinho
nascia quase centrado. O agente nunca precisou aprender a se recuperar, porque nunca
esteve em apuros.

No hardware, isso teria falhado no primeiro dedo encostado na haste.

### A correção

| Antes | Depois |
|---|---|
| ψ₀ ∈ ±3° | **ψ₀ ∈ ±10°** |
| ω₀ ∈ ±0,15 rad/s | **ω₀ ∈ ±0,8 rad/s** |
| x₀ ∈ ±30 mm | **x₀ ∈ ±120 mm** |
| sem perturbações | **empurrão aleatório a cada ~5 s** |

### Calibrando o empurrão

Na primeira tentativa usei desvio de 1,5 rad/s a cada 3 s. O treinamento travou em 0% de
sucesso. O motivo: com v_max de 0,45 m/s, chutes acima de ~1,5 rad/s **não têm solução
física**. Treinar contra o impossível só ensina o agente a desistir cedo.

Reduzi para 0,7 rad/s a cada 5 s, calibrado pelo que o hardware consegue de fato
recuperar. O treinamento voltou a andar.

### Resultado dos três agentes

| Agente | Sucesso | ⟨\|ψ\|⟩ | ⟨\|x\|⟩ | Suavidade | Recupera 10° | Empurrão 1,0 rad/s |
|---|---|---|---|---|---|---|
| v1 início estreito | 47% | 6,47° | 57 mm | **0,25** | 20% | **0%** |
| **v2 início amplo + empurrões** | **87%** | **1,98°** | **32 mm** | 1,67 | **50%** | **90%** |
| v3 início amplo, sem empurrões | 83% | 2,30° | 33 mm | 1,02 | 45% | 80% |
| PID do firmware | 23% | 7,31° | 92 mm | 0,71 | 50% | 55% |

O v2 é o recomendado. O preço da robustez é a suavidade: 1,67 contra 0,25 do v1. Se o
hardware vibrar demais, o botão a girar é o coeficiente de `|Δa|` na recompensa
(de 0,05 para 0,10 ou 0,15).

---

## 9. O teto físico

Medindo de que ângulo inicial cada controlador recupera:

| ψ₀ | Agente v2 | PID |
|---|---|---|
| 6° | 100% | 100% |
| 8° | 90% | 90% |
| 10° | 85% | 80% |
| 12° | 65% | 65% |
| 14° | 35% | 35% |
| 16° | 15% | 15% |
| 18° | 0% | 5% |

As curvas são **idênticas**. Isso não é limitação da política — é o teto do atuador.
Com v_max de 0,45 m/s, o carrinho não consegue correr para baixo da haste rápido o
bastante antes dela passar de 35°.

> Nenhum algoritmo resolve isso. Para recuperar de ângulos maiores seria preciso
> aumentar `V_MAX_PASSOS` no firmware (hoje em 18000 passos/s), e aí entraria o risco
> de perder passos.

---

## 10. As quatro métricas que revelam o que está acontecendo

| Métrica | Saudável | Se estiver errada |
|---|---|---|
| **Retorno** | sobe, com oscilação, e estabiliza | plano desde o início → recompensa mal formulada ou estado mal normalizado |
| **KL** | 0,003 a 0,02 por atualização | acima de 0,05 → passos grandes demais: reduza a taxa ou as épocas |
| **Fração recortada** | 0,05 a 0,25 | perto de 0 → o clipping nem atua. Perto de 1 → lote saturado, reduza as épocas |
| **Variância explicada** | sobe para 0,8–0,95 | negativa → o Crítico está pior que prever a média |

No treinamento deste projeto: KL ficou em 0,005, fração recortada em 0,06, variância
explicada em ~0,80. Todas na faixa saudável.

A variância explicada deu alguns mergulhos bruscos para zero — isso acontece quando a
duração dos episódios muda de repente e o Crítico precisa reaprender a escala dos
retornos. Recupera sozinho em poucas iterações.
