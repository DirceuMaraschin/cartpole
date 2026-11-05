# Pêndulo invertido controlado por PPO

Nov 5, 2026 · @Dirceu Maraschin Jr

## O que foi construído

Um pêndulo invertido físico controlado por um agente PPO treinado inteiramente em simulação, validado em bancada. Duas versões foram construídas e medidas.

|  | Agente de equilíbrio | Agente completo |
| --- | --- | --- |
| Quem faz o swing-up | controlador de energia clássico, 3,55 s | **o próprio agente, 2,47 s** |
| Quem equilibra | o agente | o agente |
| Controle clássico na solução | energia + PID | **nenhum** |
| ⟨\|ψ\|⟩ em regime | 1,46° | **0,89°** |
| Continuidade | 100% em 6,2 min, zero quedas | 90%, uma queda a cada 45 s |
| Recuperação de queda | impossível (fora de 35° o episódio acaba) | **0,81 s** |
| Validação | nove etapas de bancada, todas aprovadas | 3 × 30 s |

O agente de equilíbrio passou pelas nove etapas do roteiro de bancada e acumulou **6,2 minutos** de equilíbrio com **zero** intervenções do supervisor PID. O agente completo elimina todo o controle clássico: recebe a haste pendurada e parada, ergue-a sozinho e a mantém.

As três métricas que o simulador previa saíram **melhores** na bancada do que na previsão:

| Métrica | Simulador previu | Bancada |
| --- | --- | --- |
| ⟨\|ψ\|⟩ | 1,98° | 1,46° |
| ⟨\|x\|⟩ | 32 mm | 17,6 mm |
| sobrevive 20 s | 87% | 3 de 3 episódios de 60 s |
| swing-up até capturar | 3,60 s | 3,55 s |

Essa concordância é o resultado central do projeto, e vem da identificação do modelo a partir de logs da própria bancada — não de valores de catálogo. O agente nunca viu o equipamento antes de ser ligado a ele.

## O hardware

| Componente | Especificação |
| --- | --- |
| Microcontrolador | ESP32-S3 DevKitC-1 N8R2 (8 MB flash, 2 MB PSRAM) |
| Motor | NEMA17, 200 passos por volta, 24 V |
| Driver | DRV8825, 8 micropassos |
| Encoder | incremental, 600 pulsos por volta, quadratura |
| Transmissão | polia GT2 de 20 dentes, passo 2 mm, correia GT2 |
| Segurança | duas chaves de fim de curso, em interrupção |
| Haste | 255 mm do eixo do encoder à ponta, \~42 g, massa concentrada na ponta |
| Trilho | 600 mm de vão livre entre as chaves; carrinho de 65,5 mm |

### A aritmética da transmissão é exata

A conversão de passos para metros não foi medida, foi calculada:

- 20 dentes × 2 mm de passo = **40 mm por volta** da polia
- 200 passos × 8 micropassos = **1600 pulsos por volta**
- 40 mm ÷ 1600 = **0,025 mm por pulso** = **40 000 passos por metro**

Esse valor é exato por construção, e por isso prevalece sobre qualquer medida de régua. Três caminhos independentes concordam dentro de 0,5%: a aritmética acima (40 000, exato), a dinâmica do swing-up (39 200 ± 1200) e a régua (40 196 ± 200).

O encoder dá 600 × 4 = **2400 contagens por volta**, ou **0,15° por contagem**. Essa quantização é o piso de precisão de tudo o que vem depois: nenhuma medida de ângulo no projeto pode ser mais fina que meio passo, 0,075°.

## Identificação do modelo

Três parâmetros foram extraídos de logs da própria bancada, e são eles que explicam a transferência simulador–realidade.

| Parâmetro | Valor | De onde saiu |
| --- | --- | --- |
| L\_eq | 0,22463 m | período da oscilação livre, extrapolado para amplitude zero |
| atrito viscoso | 0,06274 1/s | envelope de decaimento da amplitude |
| atrito de Coulomb | 0,04984 rad/s² | envelope de decaimento da amplitude |

L\_eq é o comprimento de um pêndulo simples equivalente, não o comprimento da haste: a haste tem massa distribuída e massa concentrada na ponta, e o que importa para a dinâmica é a combinação. Medir a haste com régua daria o número errado.

### Método

Os dois efeitos vivem em escalas de tempo diferentes e foram separados:

1. **Período → L\_eq.** O período depende da amplitude (integral elíptica). Medindo-o em várias amplitudes e extrapolando para amplitude zero, obtém-se L\_eq sem interferência do atrito.
2. **Envelope → atrito.** A taxa de decaimento da amplitude separa o atrito viscoso (proporcional à velocidade, decaimento exponencial) do de Coulomb (constante, decaimento linear).

A janela de análise começa nas passagens por zero, onde ω é máximo: num ponto de retorno ω ≈ 0 e a quantização de 0,15° do encoder domina a medida. Um ajuste RMS direto sobre a oscilação inteira não serve — a deriva de fase passa a dominar o erro, e o otimizador converge para atrito zero.

### O modelo foi validado por três caminhos independentes

| Teste | Erro |
| --- | --- |
| Oscilação livre, 12 ciclos | 0,37° médio |
| Swing-up com o carrinho em movimento | 0,6° a 3,3° |
| Taxa de divergência perto do topo | bate com a teoria |
| Reconstrução da ação do PID a partir do log | resíduo 0,060 m/s² |

A terceira linha merece explicação. Perto do topo o pêndulo é instável: com τ = √(L/g) = 0,151 s, qualquer erro de ângulo **dobra a cada \~105 ms**. Nenhum modelo prevê essa região em malha aberta por mais de \~0,3 s; a validação possível é medir o tempo de dobra no log e confirmar que bate com a teoria.

## Controle clássico: a linha de base

Antes do PPO o pêndulo já era controlado por dois controladores clássicos em sequência, um para erguer a haste e outro para mantê-la. Ambos continuam no firmware: como linha de base de comparação e como rede de proteção do agente.

### Swing-up por energia

Ergue a haste bombeando energia a cada meio ciclo. A grandeza perseguida é a energia normalizada:

```latex
E_n = \tfrac{1}{2}\left(\frac{\omega}{\omega_0}\right)^2 - \cos\varphi, \qquad \omega_0 = \sqrt{g/L_{eq}}
```

E\_n vale −1 com a haste pendurada e parada, e +1 no topo. O controlador acelera o carrinho no sentido que aumenta E\_n até o alvo, e entrega ao PID quando a haste chega perto do topo.

**O alvo precisa ultrapassar 1,00.** Em exatamente 1,00 a trajetória é a homoclínica — a que se aproxima do topo assintoticamente e nunca o transpõe. Com alvo em 1,00, 28% das tentativas ficavam presas entre E\_n 0,90 e 1,00. Com `ALVO_EN = 1,10`, nenhuma.

O ω₀ usado aqui é **parâmetro de sintonia**, não afirmação sobre a física: o firmware usa `L_EQ_M = 0,220`, enquanto a identificação deu 0,22463.

### PID de equilíbrio

Mantém a haste depois da captura:

```latex
a = K_P\,\psi + K_D\,\dot{\psi} + K_X\,x + K_V\,v
```

com K\_P = 40, K\_D = 5,5, K\_X = 5,0 e K\_V = 17,5. Os dois primeiros termos equilibram a haste; os dois últimos centralizam o carrinho, e sem eles ele deriva até a parede.

Dois comportamentos deste controlador importam para o resto do projeto:

- **K\_X determina o erro de posição em regime.** Um offset de zero do encoder vira deslocamento permanente, x\_ss ≈ −K\_P·offset/K\_X — cerca de 47 mm por 0,1° com K\_X = 1,5. Elevar K\_X para 5,0 levou o deslocamento de 73 mm para 7 mm.
- **O produto K\_V·dt é o que limita a taxa.** A 1 kHz, K\_V = 17,5 é estável; a 50 Hz o mesmo ganho desestabiliza. Por isso o PID roda a 1 kHz no firmware, enquanto o agente decide a 50 Hz.

### O que a linha de base entrega

| Métrica | Valor medido |
| --- | --- |
| swing-up até capturar | 3,55 s |
| capturas perdidas | 0 |
| ⟨\|x\|⟩ em regime | 66 mm |

Estes são os números que o PPO precisa bater — e o controlador que assume quando o agente falha.

## PPO aplicado a este caso

O PPO treina duas redes ao mesmo tempo. O **ator** transforma o estado observado em uma ação; o **crítico** estima quanto retorno esperar daquele estado, e serve para julgar se a ação tomada foi melhor ou pior que a média.

A ideia central do PPO está no objetivo recortado. Em vez de empurrar a política o quanto o gradiente sugerir, ele limita o quanto a probabilidade de uma ação pode mudar por atualização:

```latex
L^{CLIP}(\theta) = \mathbb{E}\left[\min\left(r_t(\theta)\,\hat{A}_t,\ \mathrm{clip}\big(r_t(\theta),\,1-\epsilon,\,1+\epsilon\big)\,\hat{A}_t\right)\right]
```

onde rₜ é a razão entre a probabilidade da ação na política nova e na antiga, e Âₜ é a vantagem estimada. O recorte em 1 ± ε é o que impede uma atualização grande de destruir uma política que já funcionava — o modo de falha clássico dos métodos de gradiente de política anteriores.

A vantagem vem do GAE, que combina estimativas de vários horizontes:

```latex
\hat{A}_t = \sum_{l=0}^{T-t-1} (\gamma\lambda)^l \delta_{t+l}, \qquad \delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
```

### Hiperparâmetros, e a razão de cada um

| Parâmetro | Valor | Por que |
| --- | --- | --- |
| rede | 2 camadas de 64, tanh | o problema tem 5 ou 6 entradas; mais capacidade só aumentaria a variância |
| γ (desconto) | 0,99 (0,995 no agente completo) | horizonte efetivo de 100 passos = 2 s; o swing-up leva 175 passos e precisou de mais |
| λ (GAE) | 0,95 | compromisso usual entre viés e variância |
| ε (recorte) | 0,2 | valor de referência do artigo original |
| coef. de entropia | 0,004 (0,01 no completo) | o completo tem uma armadilha (girar sem parar) e precisa explorar mais |
| coef. de valor | 0,5 | — |
| parada por KL | 0,02 | interrompe as épocas quando a política já mudou demais |
| recorte de gradiente | 0,5 | evita um lote atípico desestabilizar o treino |
| ação | gaussiana contínua, logσ aprendido e independente do estado | aceleração é contínua; discretizar perderia resolução |

### A armadilha do truncamento

Esta é a sutileza que silenciosamente arruina implementações de PPO. Um episódio pode acabar por dois motivos distintos:

- **Terminação**: o pêndulo caiu. Não há futuro, e o valor do estado seguinte é genuinamente zero.
- **Truncamento**: o limite de passos foi atingido. O pêndulo estava equilibrado e **continuaria** — o futuro existe, só não foi simulado.

Tratar truncamento como terminação ensina ao crítico que estados perfeitamente bons valem zero, e o erro contamina o aprendizado para trás. No truncamento, o bootstrap tem de usar V(s\_T) do crítico; na terminação, zero.

### O que o agente vê

Agente de equilíbrio, 5 entradas normalizadas: posição do carrinho, velocidade do carrinho, ângulo, velocidade angular e **a ação anterior**. A última entrada existe porque o firmware aplica a ação com um passo de atraso; sem ela, o agente não teria como saber o que está em curso.

A recompensa:

```latex
r = 1 - 0{,}60\left(\frac{|\psi|}{\psi_{max}}\right)^2 - 0{,}25\left(\frac{|x|}{x_{max}}\right)^2 - 0{,}05\frac{|\Delta a|}{a_{max}}
```

com −50 ao cair. Os três termos de forma somam cerca de 0,016 em regime permanente contra 1,000 da sobrevivência: na prática o agente é pago por não cair, e o custo de uma ação nervosa é quase nulo. Isso aparece na bancada como 20,7 inversões de sinal da aceleração por segundo.

### Randomização de domínio

A cada episódio, L\_eq varia ±8%, os dois atritos de 0,5× a 1,8×, o ganho de aceleração ±10%, o zero do encoder ±0,8° e o atraso em um passo. O agente nunca treina duas vezes no mesmo mundo, e por isso não aprende a explorar imperfeições de um modelo específico.

## Arquitetura da solução

O agente roda no PC e decide; o ESP32 atua e garante os limites físicos. Essa divisão é deliberada: a rede pode errar, e nenhuma decisão dela consegue violar as saturações do firmware.

&#91;embedded content: arquitetura · PC, ESP32 e bancada, laço de 50 Hz\]

A cada 20 ms o ESP32 publica o estado, o PC responde com uma aceleração, e o firmware a aplica no ciclo seguinte — um passo de atraso, reproduzido no simulador.

### Protocolo

Serial a 921 600 baud, binário, com CRC-8/SMBUS nos dois sentidos: 21 bytes de subida (posição em passos, contagem do encoder, velocidade, velocidade angular, flags, sequência) e 7 bytes de descida (ação em int16, comando, sequência). O número de sequência detecta pacote perdido; os flags informam o modo em que o firmware está.

Texto e binário compartilham a porta, mas não o estado: em modo RL o firmware **não lê texto algum**. Essa escolha custou um bug, descrito adiante.

### Orçamento de tempo do laço

| Etapa | Tempo |
| --- | --- |
| Inferência da política | 0,04 ms |
| Ida e volta pela serial | 0,05 ms médio |
| Intervalo entre ações | 20,1 ms médio, 21,5 ms no p99 |
| Watchdog do firmware | 100 ms |

A folga parece enorme, e não é: o USB-serial do macOS entrega pacotes em lote, e lacunas de 45 a 53 ms foram medidas. Foi por isso que o watchdog subiu de 60 para 100 ms.

### Camadas de proteção

| Camada | Onde | O que faz |
| --- | --- | --- |
| Supervisor PID | firmware | \|ψ\| > 20° → o PID retoma (tem até 35° para salvar) |
| Watchdog | firmware | 100 ms sem ação → devolve ao PID ou para |
| Saturações e freio de parede | firmware | velocidade, aceleração e curso; o agente não escapa |
| Aborto pelo PC | ponte | \|ψ\| > 18° ou \|x\| > 235 mm |
| Fins de curso | ISR | cortam os pulsos na interrupção |
| Chave de emergência | o operador | em série com os 24 V |

As quatro primeiras são software e podem ter bugs — tiveram. As duas últimas, não.

## Como clássico e agente trabalham juntos

Os dois **nunca atuam ao mesmo tempo**. Não há mistura de sinais nem média ponderada: o controle é posse exclusiva, com condições definidas para trocar de mãos.

&#91;embedded content: transferência de autoridade · três arranjos\]

### Entrega: o clássico passa o bastão

O comando `r` não entrega na hora — arma uma intenção. O firmware só executa a troca quando três critérios são atendidos ao mesmo tempo:

```c
if (rlAposCaptura &&
    fabsf(psi)    < 3.0f * PI / 180.0f &&   // |psi| < 3 graus
    fabsf(psiDot) < 0.5f &&                 // |psiDot| < 0,5 rad/s
    fabsf(x)      < X_ENTREGA_RL) {         // |x| < 120 mm
```

Enquanto faltar algum, o firmware diz qual. A razão é direta: entregar um pêndulo já caindo seria pôr o agente num estado que ele mal viu no treino.

### Supervisão: o clássico vigia e pode retomar

Com o agente no comando, o PID continua calculando em silêncio a 1 kHz, sem atuar. Ele retoma em três situações:

| Gatilho | Condição | Motivo registrado |
| --- | --- | --- |
| Ângulo | \|ψ\| > 20° | `angulo acima do limite do agente` |
| Posição | \|x\| > 240 mm | `carrinho fora da faixa util` |
| Comunicação | 100 ms sem ação do PC | `comunicacao perdida` |

A retomada é abrupta e completa: `modo = EQUILIBRIO`, protocolo binário desligado, integral zerada. Os 20° de gatilho contra os 35° de limite do firmware dão ao PID **15° de margem** — e ele precisa, porque o erro dobra a cada 105 ms.

Nas nove etapas de validação esse supervisor **nunca disparou**: 6,2 minutos de equilíbrio, zero intervenções.

### Observação: ambos decidem, só o clássico atua

O comando `o` cria um terceiro estado. O firmware entra em modo RL, publica o estado a 50 Hz, recebe a ação da rede — e a descarta. Quem move o carrinho é o PID.

Serve para comparar as duas decisões no mesmo estado sem risco nenhum. Foi assim que a correlação de +0,916 foi medida, antes de a rede ter permissão de mover qualquer coisa.

### No agente completo, a cooperação acaba

Com o comando `g` o arranjo muda de natureza. O supervisor de ângulo é **desligado**, porque durante o swing-up |ψ| passa por 180° e ele dispararia no primeiro ciclo. Não há entrega: o agente recebe a haste pendurada e parada.

O que resta não é controle clássico, é limite físico — saturação de velocidade e aceleração, freio de parede e fins de curso em interrupção. Nenhum deles calcula uma ação; apenas impedem que a do agente ultrapasse o que o hardware aguenta.

A saída por falha também muda de destino: sem supervisor, perder a comunicação ou bater no trilho leva a `modo = PARADO`, não ao PID. Entregar uma haste possivelmente pendurada a um PID de equilíbrio seria pior que parar.

## Agente 1: equilíbrio

Este agente recebe a haste já erguida pelo controlador de energia clássico e a mantém. Foi validado em nove etapas, cada uma com critério de aprovação definido **antes** de rodar.

| Etapa | O que testa | Critério | Resultado |
| --- | --- | --- | --- |
| 0 | o PID continua funcionando após a recompilação | swing-up captura | 3,6 s na primeira tentativa |
| 1 | velocidade em 100% | fator 1,0 | `vmax=0,450` confirmado |
| 2 | ponte e simulador veem o mesmo estado | erro < meio passo | x 0,0125 mm, ψ 0,063° |
| 3 | o agente decide, o PID atua | mesmo sinal > 60%, correlação > 0,3 | **80% e +0,916** |
| 4 | primeiro episódio real | 10 s sem supervisor | 30 s, zero intervenções |
| 5 | supervisor desligado no firmware | 20 s sozinho | 20 s, ⟨\|ψ\|⟩ 1,53° |
| 6 | perda de passos | sem deriva após 3 episódios | nenhuma |
| 7 | desempenho real | comparar com o simulador | 3 × 60 s completos |
| 8 | perturbação | recupera de toques leves | 24 de 24 toques |

### O teste de observação

O agente decide e o PID atua. Nada se move por ordem da rede, e mesmo assim o teste responde à pergunta mais perigosa do projeto: **o sinal está invertido em algum ponto da cadeia?** Se `ENC_SINAL`, a convenção de ψ ou a direção do motor estiverem trocados, o agente decidiria sistematicamente no sentido contrário ao do PID — e empurraria o carrinho para o lado da queda. A correlação de **+0,916** entre as duas decisões descarta isso antes de qualquer movimento comandado pela rede.

### A comparação que importa

| Métrica | Simulador previu | Bancada (8.952 amostras) |
| --- | --- | --- |
| ⟨\|ψ\|⟩ | 1,98° | **1,46°** (rms 1,9°, máx 8,85°) |
| ⟨\|x\|⟩ | 32 mm | **17,6 mm** |
| sobrevive 20 s | 87% | **3 de 3 × 60 s** |

Deriva de posição nos três episódios: −0,02, −0,16 e +0,14 mm/s — zero dentro do ruído, o que confirma a etapa 6 por um caminho independente. Um único pacote perdido em 8.952.

## Agente 2: swing-up e equilíbrio, sem controle clássico

Este agente recebe a haste pendurada e parada, ergue-a sozinho e a mantém. Não há PID nem controlador de energia em momento algum.

### Quatro mudanças, cada uma obrigatória

**1. O ângulo passa a ser sin(ψ) e cos(ψ).** A observação antiga entregava ψ/35°. Com a haste pendurada ψ = 180°, o que dá **5,14** numa entrada que nunca passou de ±1 no treino. Pior: ψ/35° é descontínuo na volta completa — +180° e −80° são o mesmo estado físico e dariam +5,14 e −5,14. sin/cos é contínuo e único em toda a rotação. Como a rede passa de 5 para 6 entradas, isso obriga a treinar do zero.

**2. Sem terminação por ângulo.** Antes o episódio morria com −50 quando |ψ| passava de 35°. A região do swing-up era, para o agente, literalmente o fim do mundo. Agora só o trilho termina o episódio.

**3. Recompensa por altura, não por proximidade do equilíbrio.** (1 + cos ψ)/2 vale 0 pendurado e 1 em pé, e dá gradiente em toda a rotação — é o que faz o agente descobrir que precisa bombear energia.

**4. Escala de ψ̇ de 6 para 15 rad/s.** Para transpor o topo a haste passa pelo fundo a 13,5 rad/s. Em 6,0 a observação saturaria 2,3× durante toda a subida. O pico medido em bancada foi 13,86 rad/s — a escala de 15 não saturou.

### Como ele sobe

&#91;embedded content: logs/sw/01.csv · 3,2 s iniciais, amostrado a cada 80 ms\]

A escada é a história: cada patamar é meio ciclo de oscilação, cada degrau é um impulso do carrinho. Quatro impulsos, com inversão de direção do carrinho a cada um, usando 254 mm de curso. No quarto, a energia ultrapassa o topo (En = 1,10) e o agente a captura exatamente em **En = 0,99** — energia de estar em pé e parado.

O controlador de energia clássico persegue `ALVO_EN = 1,10` por sintonia manual. O agente chegou à mesma estratégia sozinho, e captura mais cedo.

### Resultado de bancada

| Métrica | Simulador previu | Bancada (3 × 30 s) | Clássico |
| --- | --- | --- | --- |
| swing-up | 2,87 s | **2,47 s** (idêntico nos três) | 3,55 s |
| fica em pé | 88% | **90,0%** | — |
| ⟨\|ψ\|⟩ em pé | 0,73° | **0,89°** | 1,46° (agente 1) |
| x máximo | 230 mm | **178 mm** | — |
| quedas | 1,1 por 30 s | 2 em 90 s | 0 |
| recuperação de queda | — | **0,81 s** | impossível |

O bit `F_COMPLETO` esteve aceso em 100% das amostras: o PID não tocou no pêndulo.

### Cair deixou de ser falha

O agente cai cerca de uma vez a cada 45 s, dá a volta completa e refaz o swing-up em 0,81 s. Isso muda o que "recuperar" significa: em simulação ele recupera **95%** dos chutes de 3,0 rad/s, contra **0%** do agente 1 a 1,5 rad/s. Não é um controlador melhor perto do equilíbrio — é um controlador para quem perder a haste é recuperável.

## Como reproduzir

### Os arquivos

| Arquivo | O que é |
| --- | --- |
| `pendulo_env.py` | simulador: a física identificada, única fonte dos parâmetros |
| `pendulo_env_sw.py` | o mesmo, com o problema do swing-up (importa a física de cima) |
| `ppo.py` | ator-crítico, buffer com GAE, atualização recortada |
| `treinar.py` / `treinar_sw.py` | treino de cada agente |
| `avaliar.py` / `avaliar_sw.py` | baterias de validação em simulação |
| `rl_bridge.py` | ponte PC–ESP32: protocolo, política em numpy, proteções |
| `firmware/classico/pendulo_pid.ino` | controle clássico puro: swing-up por energia + PID, sem PC |
| `firmware/pendulo_rl.ino` | firmware com modo RL e modo agente completo |
| `firmware/compila_teste/` | compila o `.ino` com g++, sem a IDE |
| `roteiro_bancada.md` | as nove etapas, com critérios e resultados |
| `identificacao.md` | a identificação do modelo, passo a passo |

### A ordem

1. **Gravar o firmware** e confirmar que o controle clássico funciona. Sem isso, nada do resto é interpretável.
2. **Registrar logs** de oscilação livre e de swing-up.
3. **Identificar o modelo** desses logs: período para L\_eq, envelope para o atrito.
4. **Validar o simulador** contra os logs antes de treinar qualquer coisa.
5. **Treinar**, com randomização de domínio e distribuição inicial larga.
6. **Rodar os testes sem hardware** — todos passam antes de a bancada ser ligada.
7. **Seguir o roteiro de bancada** na ordem, sem pular critério.

O passo 1 usa `firmware/classico/pendulo_pid.ino`, que roda **sem o PC**: Monitor Serial a 115200, depois `z` com a haste pendurada e parada, e `i` para o swing-up. É a linha de base dos 3,55 s. Suas constantes de controle são as mesmas do `pendulo_rl.ino` — o firmware de RL não tem física diferente, tem o modo RL a mais.

### Os comandos

Testes sem hardware, antes de qualquer episódio:

```
python teste_ponte.py && python teste_ponte_sw.py && python teste_auto.py
python teste_travada.py && python teste_completo.py
bash firmware/compila_teste/compila.sh
```

Agente de equilíbrio, com o PID fazendo o swing-up:

```
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt \
       --segundos 60 --log logs/rl_01.csv --auto --sem-observacao
```

Agente completo, sem controle clássico:

```
python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos_sw/melhor.pt \
       --segundos 30 --log logs/sw_01.csv --auto --completo
```

Treino do agente completo, cerca de 1,3 h:

```
python treinar_sw.py --saida modelos_sw --nucleos 1
```

Mais núcleos não ajudam: 86% do tempo é a simulação física, um laço Python sequencial de 20 sub-passos de 1 ms por passo do agente. Só vetorizar vários ambientes em paralelo aceleraria.
