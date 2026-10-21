"""
PPO (Proximal Policy Optimization) para o pêndulo invertido físico.

Implementação enxuta e comentada. Cada bloco corresponde a uma parte da teoria:

    Ator      -> a política: dado o estado, que aceleração aplicar
    Crítico   -> a função de valor: quão boa é esta situação
    GAE       -> quanto cada ação foi melhor que a média esperada
    Clipping  -> impede que uma única atualização estrague a política

Depende apenas de torch e numpy.
"""

import numpy as np
import torch
import torch.nn as nn


# =====================================================================
# 1. AS DUAS REDES
# =====================================================================
def _camada(entrada, saida, ganho=np.sqrt(2)):
    """Camada linear com inicialização ortogonal.

    A inicialização ortogonal é o padrão nas implementações de referência do
    PPO: ela mantém a escala dos sinais estável ao atravessar as camadas, o que
    na prática produz um treinamento visivelmente menos errático do que a
    inicialização aleatória comum.
    """
    cam = nn.Linear(entrada, saida)
    nn.init.orthogonal_(cam.weight, ganho)
    nn.init.constant_(cam.bias, 0.0)
    return cam


class AtorCritico(nn.Module):
    """
    Duas redes pequenas, 2 camadas ocultas de 64 neurônios com tanh.

    Por que tanh e não ReLU: tanh satura suavemente em [-1, 1], o que evita
    saídas explosivas quando o estado sai da faixa vista no treinamento —
    situação garantida no hardware real. ReLU treina mais rápido em visão
    computacional, mas em controle contínuo o tanh é mais estável.

    Por que 64x64: o problema tem 5 entradas e 1 saída. Redes maiores não
    melhoram nada aqui e só tornam a inferência mais lenta — e a inferência
    acontece dentro do laço de 20 ms.

    Por que redes SEPARADAS (e não um tronco compartilhado): o Ator precisa de
    precisão perto do equilíbrio; o Crítico precisa prever números grandes
    (retornos de até ~900). Compartilhar camadas faz um gradiente atrapalhar o
    outro. Com redes deste tamanho, o custo de separar é irrelevante.
    """

    def __init__(self, dim_obs=5, dim_acao=1, oculta=64, log_std_inicial=-0.5):
        super().__init__()

        self.ator = nn.Sequential(
            _camada(dim_obs, oculta), nn.Tanh(),
            _camada(oculta, oculta), nn.Tanh(),
            _camada(oculta, dim_acao, ganho=0.01),   # ganho pequeno: começa quase parado
        )

        self.critico = nn.Sequential(
            _camada(dim_obs, oculta), nn.Tanh(),
            _camada(oculta, oculta), nn.Tanh(),
            _camada(oculta, 1, ganho=1.0),
        )

        # Desvio-padrão da política, em log. É um PARÂMETRO TREINÁVEL, não
        # depende do estado: o agente aprende sozinho o quanto ainda precisa
        # explorar, e esse valor cai naturalmente conforme ele fica confiante.
        # Começa em exp(-0.5) = 0.61, ou seja, bastante exploração no início.
        self.log_std = nn.Parameter(torch.full((dim_acao,), float(log_std_inicial)))

    def distribuicao(self, obs):
        """A política é uma gaussiana: média vinda da rede, desvio aprendido."""
        media = self.ator(obs)
        std = torch.exp(self.log_std).expand_as(media)
        return torch.distributions.Normal(media, std)

    def avaliar(self, obs, acao):
        """Usado na otimização: log-probabilidade, entropia e valor."""
        dist = self.distribuicao(obs)
        logp = dist.log_prob(acao).sum(-1)
        ent = dist.entropy().sum(-1)
        valor = self.critico(obs).squeeze(-1)
        return logp, ent, valor

    @torch.no_grad()
    def agir(self, obs, deterministico=False):
        """Usado na coleta e na execução. Devolve ação, log-prob e valor."""
        obs_t = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
        dist = self.distribuicao(obs_t)
        # Na coleta a ação é SORTEADA (exploração).
        # Na execução final usamos a média (determinístico, sem tremor).
        acao = dist.mean if deterministico else dist.sample()
        logp = dist.log_prob(acao).sum(-1)
        valor = self.critico(obs_t).squeeze(-1)
        return (acao.squeeze(0).numpy(),
                float(logp.item()),
                float(valor.item()))


# =====================================================================
# 2. O BUFFER DE EXPERIÊNCIAS
# =====================================================================
class Buffer:
    """Guarda um lote de passos e calcula as vantagens pelo GAE."""

    def __init__(self, n_passos, dim_obs, dim_acao, gamma=0.99, lam=0.95):
        self.obs = np.zeros((n_passos, dim_obs), np.float32)
        self.aco = np.zeros((n_passos, dim_acao), np.float32)
        self.logp = np.zeros(n_passos, np.float32)
        self.rec = np.zeros(n_passos, np.float32)
        self.val = np.zeros(n_passos, np.float32)
        self.fim = np.zeros(n_passos, np.float32)   # 1 se o episódio acabou aqui
        self.corte = np.zeros(n_passos, np.float32) # 1 se foi truncado (tempo esgotado)
        self.val_prox = np.zeros(n_passos, np.float32)
        self.n = n_passos
        self.gamma, self.lam = gamma, lam
        self.i = 0

    def guardar(self, obs, aco, logp, rec, val, fim, corte, val_prox):
        k = self.i
        self.obs[k] = obs; self.aco[k] = aco; self.logp[k] = logp
        self.rec[k] = rec; self.val[k] = val
        self.fim[k] = fim; self.corte[k] = corte; self.val_prox[k] = val_prox
        self.i += 1

    def finalizar(self):
        """
        GAE — Generalized Advantage Estimation.

            delta_t = r_t + gamma * V(s_{t+1}) - V(s_t)
            A_t     = delta_t + gamma * lambda * A_{t+1}

        delta_t é a "surpresa": o quanto a situação ficou melhor do que o
        Crítico esperava. A recursão de trás para frente soma as surpresas
        futuras com peso decrescente (gamma*lambda)^k.

        Detalhe que quase todo mundo erra: episódio TRUNCADO (tempo esgotado)
        não é o mesmo que episódio TERMINADO (caiu). No truncado o futuro
        existe, só paramos de olhar — por isso usamos V(s_final) como
        estimativa (bootstrap). Sem isso o agente aprende que sobreviver até o
        fim é ruim, porque o retorno some de repente.
        """
        vant = np.zeros(self.n, np.float32)
        acum = 0.0
        for t in reversed(range(self.n)):
            continua = 1.0 - self.fim[t]           # 0 se caiu de verdade
            v_prox = self.val_prox[t]
            delta = self.rec[t] + self.gamma * v_prox * continua - self.val[t]
            # a recursão quebra tanto ao cair quanto ao truncar
            quebra = 1.0 - max(self.fim[t], self.corte[t])
            acum = delta + self.gamma * self.lam * quebra * acum
            vant[t] = acum
        self.vant = vant
        self.ret = vant + self.val                 # alvo para treinar o Crítico
        self.i = 0

    def lotes(self, tam_lote, rng):
        """Embaralha e devolve minilotes."""
        idx = rng.permutation(self.n)
        for k in range(0, self.n, tam_lote):
            j = idx[k:k + tam_lote]
            yield (torch.as_tensor(self.obs[j]),
                   torch.as_tensor(self.aco[j]),
                   torch.as_tensor(self.logp[j]),
                   torch.as_tensor(self.vant[j]),
                   torch.as_tensor(self.ret[j]))


# =====================================================================
# 3. O AGENTE PPO
# =====================================================================
class PPO:
    def __init__(self, dim_obs=5, dim_acao=1,
                 lr=3e-4, gamma=0.99, lam=0.95, eps_clip=0.2,
                 c_valor=0.5, c_entropia=0.004,
                 epocas=10, tam_lote=64, n_passos=4096,
                 kl_alvo=0.02, max_grad=0.5, semente=0):

        torch.manual_seed(semente)
        self.rng = np.random.default_rng(semente)

        self.rede = AtorCritico(dim_obs, dim_acao)
        self.otim = torch.optim.Adam(self.rede.parameters(), lr=lr, eps=1e-5)

        self.gamma, self.lam = gamma, lam
        self.eps = eps_clip
        self.c_valor, self.c_ent = c_valor, c_entropia
        self.epocas, self.tam_lote = epocas, tam_lote
        self.n_passos = n_passos
        self.kl_alvo, self.max_grad = kl_alvo, max_grad
        self.dim_obs, self.dim_acao = dim_obs, dim_acao

    # ---------------- coleta ----------------
    def coletar(self, env, estado):
        """Roda a política atual por n_passos. NÃO treina nada aqui."""
        buf = Buffer(self.n_passos, self.dim_obs, self.dim_acao, self.gamma, self.lam)
        obs = estado
        retornos_ep, duracoes_ep = [], []
        ret_atual, dur_atual = 0.0, 0

        for _ in range(self.n_passos):
            acao, logp, valor = self.rede.agir(obs)
            obs2, rec, caiu, corte, _ = env.step(acao)
            ret_atual += rec; dur_atual += 1

            # valor do próximo estado, necessário para o delta do GAE
            if caiu:
                v_prox = 0.0
            else:
                _, _, v_prox = self.rede.agir(obs2)

            buf.guardar(obs, acao, logp, rec, valor,
                        float(caiu), float(corte), v_prox)

            if caiu or corte:
                retornos_ep.append(ret_atual)
                duracoes_ep.append(dur_atual)
                ret_atual, dur_atual = 0.0, 0
                obs = env.reset()
            else:
                obs = obs2

        buf.finalizar()
        return buf, obs, retornos_ep, duracoes_ep

    # ---------------- otimização ----------------
    def atualizar(self, buf):
        """O coração do PPO: K épocas sobre o mesmo lote, com clipping."""
        # Normalizar as vantagens coloca-as em média 0 e desvio 1. Sem isso o
        # tamanho efetivo do passo de gradiente muda junto com a escala da
        # recompensa, e qualquer ajuste na recompensa exigiria re-sintonizar a
        # taxa de aprendizado.
        v = buf.vant
        buf.vant = (v - v.mean()) / (v.std() + 1e-8)

        kls, fracs, perdas_a, perdas_c, ents = [], [], [], [], []
        parou_em = self.epocas

        for ep in range(self.epocas):
            kl_epoca = []
            for obs, aco, logp_velho, vant, ret in buf.lotes(self.tam_lote, self.rng):

                logp, ent, valor = self.rede.avaliar(obs, aco)

                # razão entre a política nova e a antiga para a MESMA ação.
                # Trabalhamos em log e exponenciamos: mais estável que dividir
                # probabilidades diretamente.
                razao = torch.exp(logp - logp_velho)

                # --- objetivo clipado ---
                # O min() entre o termo normal e o recortado torna o objetivo
                # PESSIMISTA: o PPO sempre adota a estimativa mais conservadora.
                obj1 = razao * vant
                obj2 = torch.clamp(razao, 1 - self.eps, 1 + self.eps) * vant
                perda_ator = -torch.min(obj1, obj2).mean()

                # --- crítico: erro quadrático contra o retorno observado ---
                perda_critico = ((valor - ret) ** 2).mean()

                # --- entropia: paga o agente para continuar explorando ---
                perda_ent = -ent.mean()

                perda = (perda_ator
                         + self.c_valor * perda_critico
                         + self.c_ent * perda_ent)

                self.otim.zero_grad()
                perda.backward()
                # segunda rede de proteção, independente do clipping da razão
                nn.utils.clip_grad_norm_(self.rede.parameters(), self.max_grad)
                self.otim.step()

                with torch.no_grad():
                    # estimador de KL de Schulman: mais estável que -(logp-logp_velho)
                    d = logp_velho - logp
                    kl = torch.mean(torch.exp(d) - 1 - d).item()
                    frac = torch.mean(((razao - 1).abs() > self.eps).float()).item()
                kl_epoca.append(kl); fracs.append(frac)
                perdas_a.append(perda_ator.item())
                perdas_c.append(perda_critico.item())
                ents.append(ent.mean().item())

            kl_med = float(np.mean(kl_epoca)); kls.append(kl_med)

            # PARADA ANTECIPADA POR KL.
            # Se a política já se afastou demais da que gerou os dados, as
            # épocas restantes só fariam extrapolação. No hardware isso importa
            # mais ainda: uma política ruim causa dano físico.
            if kl_med > 1.5 * self.kl_alvo:
                parou_em = ep + 1
                break

        return {
            'kl': float(np.mean(kls)),
            'clip_frac': float(np.mean(fracs)),
            'perda_ator': float(np.mean(perdas_a)),
            'perda_critico': float(np.mean(perdas_c)),
            'entropia': float(np.mean(ents)),
            'epocas_usadas': parou_em,
            'std': float(torch.exp(self.rede.log_std).mean().item()),
        }

    # ---------------- persistência ----------------
    def salvar(self, caminho):
        torch.save({'rede': self.rede.state_dict()}, caminho)

    def carregar(self, caminho):
        d = torch.load(caminho, map_location='cpu', weights_only=True)
        self.rede.load_state_dict(d['rede'])


def variancia_explicada(valores, retornos):
    """Diagnóstico do Crítico. 1,0 = perfeito. <= 0 = pior que prever a média."""
    var = np.var(retornos)
    return float('nan') if var == 0 else float(1 - np.var(retornos - valores) / var)
