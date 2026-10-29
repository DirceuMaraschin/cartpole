"""
Ambiente de simulação do pêndulo invertido físico (ESP32 + NEMA17 + GT2 + encoder 600 PPR).

Todos os parâmetros foram IDENTIFICADOS a partir dos logs reais do equipamento,
não copiados do CartPole padrão. Veja identificacao.md para a origem de cada valor.

O ambiente replica, de propósito, as imperfeições do hardware:
  - quantização do encoder (0,15 graus por contagem)
  - o mesmo filtro de omega do firmware (janela de 8 amostras + EMA 50%)
  - atraso de atuação (a ação só tem efeito no passo seguinte)
  - saturação de velocidade e o freio de parede do firmware
  - integração interna a 1 kHz, com o agente decidindo a 50 Hz

Interface compatível com Gymnasium (reset/step), sem exigir a biblioteca.
"""

import numpy as np

# =====================================================================
# PARÂMETROS IDENTIFICADOS (logs de 20/10/2026)
# =====================================================================
G = 9.81

# --- pêndulo: da oscilação livre de 72 s em livre_01.csv ---
# Conferência física: haste de 255 mm com ~47% da massa na ponta dá L_eq = 224,6 mm.
# (haste uniforme daria 170 mm; massa toda na ponta daria 255 mm)
# A massa de 42 g NÃO entra na dinâmica: o motor de passo impõe a trajetória do
# carrinho independentemente da reação do pêndulo. Ela só importaria para conferir
# torque — e 42 g pedem ~0,4 N contra os ~39 N disponíveis, folga de 100x.
L_EQ    = 0.22463    # m    comprimento equivalente (firmware usava 0,220)
B_VISC  = 0.06274    # 1/s  atrito viscoso no pivô
C_COUL  = 0.04984    # rad/s^2  atrito seco (Coulomb) no pivô
OM_EPS  = 0.02       # rad/s  suavização do sinal do atrito seco

# --- trilho e atuador ---
# PENDENTE: a régua (600 mm entre fins de curso) implica 35800 passos/m, mas a
# dinâmica do swing-up mede 39200 +-1200. Divergência de 9%, ainda não resolvida.
# Até a calibração direta (ver identificacao.md secao 6), fica o valor teórico e
# a randomização de escala vai a +-8% para cobrir a incerteza.
PASSOS_POR_M = 40000.0
CURSO_M      = 0.5329       # curso útil pelo homing, NA ESCALA TEÓRICA
X_LIM_FIS    = CURSO_M/2    # +-0,2665 m da extremidade física
V_MAX        = 0.45         # m/s   = 18000 passos/s / 40000
A_MAX        = 8.0          # m/s^2 saturação da ação (A_BAL_MAX do firmware)
A_FREIO      = 8.0          # m/s^2 freio de parede
MARGEM_M     = 0.01         # folga do freio de parede

# --- sensor ---
PASSO_ENC = 2*np.pi/2400    # rad por contagem = 0,15 graus

# --- tempos ---
DT_FIS  = 0.001             # integração interna, igual ao laço do firmware
DT_CTRL = 0.020             # passo do agente: 50 Hz
N_SUB   = int(round(DT_CTRL/DT_FIS))

# --- limites do episódio ---
PSI_LIM = np.radians(35.0)  # SAIDA_GRAUS do firmware
X_LIM   = 0.24              # limite de software, dentro do físico (0,2665)

# --- escalas de normalização da observação ---
PSID_ESC = 6.0              # rad/s


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class PenduloInvertido:
    """
    Estado interno (verdadeiro):   x, v, phi, omega
    Observação entregue ao agente: [x, v, psi, psiDot, a_anterior] normalizados

    phi  = 0 com a haste pendurada, pi com a haste em pé (convenção do firmware)
    psi  = wrap(pi - phi)  -> inclinação em relação à vertical, 0 = equilibrado
    psiDot = -omega
    """

    def __init__(self, dominio_aleatorio=True, atraso_passos=1, semente=None,
                 ruido_encoder=True, max_passos=1000,
                 inicio_amplo=True, perturbacoes=True):
        self.rng = np.random.default_rng(semente)
        self.dominio_aleatorio = dominio_aleatorio
        self.atraso_passos = atraso_passos
        self.ruido_encoder = ruido_encoder
        self.max_passos = max_passos
        # inicio_amplo: sorteia estados iniciais bem fora do equilíbrio.
        #   SEM isso o agente aprende a controlar apenas em torno de 0 e nunca
        #   aprende a RECUPERAR. Foi exatamente a falha pega pelo teste 5.
        self.inicio_amplo = inicio_amplo
        # perturbacoes: empurrões aleatórios durante o episódio, equivalentes a
        #   um dedo batendo na haste. Ensina recuperação em regime, não só na
        #   largada.
        self.perturbacoes = perturbacoes
        self.dim_obs = 5
        self.dim_acao = 1          # contínua, em [-1, 1] -> escala para +-A_MAX

    # ---------------- randomização de domínio ----------------
    def _sortear_mundo(self):
        r = self.rng
        if self.dominio_aleatorio:
            self.L   = L_EQ   * r.uniform(0.92, 1.08)
            self.b   = B_VISC * r.uniform(0.5, 1.8)
            self.c   = C_COUL * r.uniform(0.5, 1.8)
            self.ganho_a = r.uniform(0.90, 1.10)     # correia/escala de passos
            self.off_psi = r.normal(0, np.radians(0.8))
            self.atraso  = self.atraso_passos + r.integers(0, 2)
            self.escala_x = r.uniform(0.92, 1.08)    # erro de PASSOS_POR_M (ver nota acima)
        else:
            self.L, self.b, self.c = L_EQ, B_VISC, C_COUL
            self.ganho_a, self.off_psi = 1.0, 0.0
            self.atraso, self.escala_x = self.atraso_passos, 1.0

    # ---------------- filtro de omega igual ao do firmware ----------------
    def _reset_filtro(self):
        self._hist = [self.phi]*8
        self._om_f = 0.0

    def _filtrar_omega(self, phi_medido):
        antigo = self._hist.pop(0)
        self._hist.append(phi_medido)
        bruto = (phi_medido - antigo)/(8*DT_FIS)
        self._om_f = 0.5*self._om_f + 0.5*bruto
        return self._om_f

    # ---------------- ciclo do ambiente ----------------
    def reset(self, psi0=None, x0=None):
        self._sortear_mundo()
        r = self.rng
        if psi0 is None:
            lim = np.radians(10.0) if self.inicio_amplo else np.radians(3.0)
            psi = r.uniform(-lim, lim)
        else:
            psi = psi0
        self.phi = np.pi - psi
        self.omega = r.uniform(-0.8, 0.8) if self.inicio_amplo else r.uniform(-0.15, 0.15)
        if x0 is None:
            lim_x = 0.12 if self.inicio_amplo else 0.03
            self.x = r.uniform(-lim_x, lim_x)
        else:
            self.x = x0
        self.v = 0.0
        self.a_cmd = 0.0
        self.fila = [0.0]*max(1, self.atraso)
        self.passos = 0
        self._reset_filtro()
        self._phi_q = self.phi
        return self._observar()

    def _medir(self):
        """aplica quantização do encoder e o filtro, como no hardware"""
        if self.ruido_encoder:
            self._phi_q = np.round(self.phi/PASSO_ENC)*PASSO_ENC
        else:
            self._phi_q = self.phi
        return self._phi_q

    def _observar(self):
        psi  = _wrap(np.pi - self._phi_q) + self.off_psi
        psid = -self._om_f
        return np.array([
            self.x*self.escala_x / X_LIM,
            self.v / V_MAX,
            psi / PSI_LIM,
            psid / PSID_ESC,
            self.a_cmd / A_MAX,
        ], dtype=np.float32)

    def step(self, acao):
        # ação contínua em [-1,1] -> aceleração pedida
        a_pedida = float(np.clip(np.asarray(acao).reshape(-1)[0], -1.0, 1.0)) * A_MAX
        a_ant = self.a_cmd
        self.a_cmd = a_pedida

        # atraso de atuação: o que atua agora foi decidido há N passos
        self.fila.append(a_pedida)
        a_efetiva = self.fila.pop(0) * self.ganho_a

        # Empurrão aleatório: ~1 a cada 250 passos (5 s).
        # O desvio de 0,7 rad/s foi calibrado pelo que é FISICAMENTE recuperável:
        # com v_max de 0,45 m/s, chutes acima de ~1,5 rad/s não têm solução, e
        # treinar com o impossível só ensina o agente a desistir cedo.
        if self.perturbacoes and self.rng.random() < 1/250:
            self.omega += self.rng.normal(0, 0.7)

        # ---- integra N_SUB passos de 1 ms, igual ao firmware ----
        for _ in range(N_SUB):
            # cinemática do carrinho: v integra a, com saturações do firmware
            v_bruta = self.v + a_efetiva*DT_FIS
            self.v = np.clip(v_bruta, -V_MAX, V_MAX)

            # freio de parede: nunca mais rápido do que consegue frear
            dB = max(0.0, (X_LIM_FIS - self.x) - MARGEM_M)
            dA = max(0.0, (self.x + X_LIM_FIS) - MARGEM_M)
            self.v = min(self.v, np.sqrt(2*A_FREIO*dB))
            self.v = max(self.v, -np.sqrt(2*A_FREIO*dA))

            a_real = (self.v - (v_bruta - a_efetiva*DT_FIS))/DT_FIS
            self.x += self.v*DT_FIS

            # dinâmica do pêndulo com acoplamento do carrinho
            acc = ((-G*np.sin(self.phi) - a_real*np.cos(self.phi))/self.L
                   - self.b*self.omega
                   - self.c*np.tanh(self.omega/OM_EPS))
            self.omega += acc*DT_FIS
            self.phi   += self.omega*DT_FIS

            self._filtrar_omega(self._medir())

        self.passos += 1
        obs = self._observar()

        # ---- recompensa ----
        psi_v = _wrap(np.pi - self.phi)          # usa o valor verdadeiro
        caiu  = abs(psi_v) > PSI_LIM or abs(self.x) > X_LIM
        trunc = self.passos >= self.max_passos

        # Coeficientes revisados em 29/10 a partir da bancada. Os valores
        # antigos (0.60 / 0.25 / 0.05) produziam, em regime permanente:
        #   sobrevivencia 1.000 | psi 0.001 | x 0.001 | Delta-a 0.014
        # ou seja, o agente era pago quase exclusivamente por nao cair, e
        # tremular nao custava NADA. Resultado medido na bancada: 20,7
        # inversoes de sinal da aceleracao por segundo e |da|/|a| = 1,14.
        #
        # Duas mudancas:
        #  - Delta-a de 0.05 para 0.50: agora a suavidade pesa ~14% do bonus
        #    de sobrevivencia no nivel de tremulacao antigo, e o gradiente e
        #    linear (o quadratico desaparece perto de zero, que e justamente
        #    onde queremos empurrar).
        #  - termo LINEAR de posicao: o quadratico torna x pequeno quase
        #    gratuito, e o agente so reagia quando x ja estava grande --
        #    quando o limite de velocidade (0,45 m/s) ja impedia recuperar.
        #    Foi assim que o carrinho fugiu para -233 mm na etapa 8.
        r = (1.0
             - 0.60*(abs(psi_v)/PSI_LIM)**2
             - 0.25*(abs(self.x)/X_LIM)**2
             - 0.15*(abs(self.x)/X_LIM)
             - 0.50*abs(self.a_cmd - a_ant)/A_MAX)
        if caiu:
            r = -50.0

        info = {'psi': psi_v, 'x': self.x, 'v': self.v, 'a': self.a_cmd}
        return obs, r, caiu, trunc, info

    # ---------------- utilidades ----------------
    def estado_verdadeiro(self):
        return np.array([self.x, self.v, _wrap(np.pi - self.phi), -self.omega])


# =====================================================================
# O PID DO FIRMWARE, para servir de referência dentro do simulador
# =====================================================================
class PIDFirmware:
    """Reproduz exatamente o controlador do testependulo.ino."""
    KP, KI, KD, KX, KV = 40.0, 1.0, 5.5, 1.5, 17.5
    I_MAX = 0.5

    def __init__(self):
        self.termoI = 0.0

    def reset(self):
        self.termoI = 0.0

    def __call__(self, obs, dt=DT_CTRL):
        x    = obs[0]*X_LIM
        v    = obs[1]*V_MAX
        psi  = obs[2]*PSI_LIM
        psid = obs[3]*PSID_ESC
        base = self.KP*psi + self.KD*psid + self.KX*x + self.KV*v
        cand = np.clip(self.termoI + self.KI*psi*dt, -self.I_MAX, self.I_MAX)
        bruto = base + cand
        saturou = abs(bruto) > A_MAX
        if (not saturou) or (bruto > A_MAX and psi < 0) or (bruto < -A_MAX and psi > 0):
            self.termoI = cand
        a = np.clip(base + self.termoI, -A_MAX, A_MAX)
        return a / A_MAX        # devolve no formato de ação do ambiente
