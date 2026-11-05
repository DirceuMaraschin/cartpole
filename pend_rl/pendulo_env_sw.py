"""
Ambiente para o agente fazer TUDO: swing-up E equilibrio, sem controle clássico.

A FÍSICA É IDÊNTICA a pendulo_env.py — os mesmos L_eq, atrito viscoso e de
Coulomb identificados dos logs, o mesmo filtro de omega do firmware, a mesma
quantização do encoder, o mesmo atraso e as mesmas saturações. Nada disso muda,
porque foi justamente a fidelidade da física que entregou a transferência
simulador->bancada das etapas 4 a 8 (previsão 1,98 graus, bancada 1,46).

O que muda é só o PROBLEMA proposto ao agente. Quatro alterações, cada uma
necessária por um motivo específico:

1. OBSERVAÇÃO com sin(psi) e cos(psi) em vez de psi/PSI_LIM.
   A antiga entrega `psi / 35 graus`. Com a haste pendurada psi = 180 graus, o
   que dá 5,14 numa entrada que nunca passou de +-1 no treino — a rede não tem
   comportamento definido ali. Pior: psi/35 é DESCONTÍNUO na volta completa
   (+180 e -180 graus são o mesmo estado físico e dariam +5,14 e -5,14).
   sin/cos é contínuo em toda a rotação e único para cada ângulo.
   Isto obriga a treinar do zero: a rede tem 6 entradas, não 5.

2. SEM TERMINAÇÃO por ângulo.
   Antes o episódio morria com -50 quando |psi| passava de 35 graus. A região
   do swing-up era, para o agente, literalmente o fim do mundo. Agora só o
   trilho termina o episódio.

3. ESCALA DE psiDot de 6 para 15 rad/s.
   Para transpor o topo com En = 1,10 o pêndulo passa pelo fundo a
   W0*sqrt(2*(En+1)) = 13,5 rad/s, com W0 = sqrt(g/L_eq) = 6,61. Em 6,0 a
   observação saturaria em 2,3x durante todo o swing-up.

4. RECOMPENSA por ALTURA, não por proximidade do equilíbrio.
   (1+cos(psi))/2 vale 0 pendurado e 1 em pé, e dá gradiente em TODA a
   rotação — é o que faz o agente descobrir que precisa bombear energia.
   Sozinho esse termo premia girar sem parar (altura média 0,5), então vem
   acompanhado de uma penalidade de |psiDot| e de um bônus de precisão perto
   do topo. Os valores estão conferidos em `_valores_de_referencia()`.
"""

import numpy as np

# A física vem toda do ambiente validado: um único lugar para os parâmetros
# identificados, para não haver chance de divergirem.
from pendulo_env import (
    G, L_EQ, B_VISC, C_COUL, OM_EPS,
    PASSOS_POR_M, CURSO_M, X_LIM_FIS, V_MAX, A_MAX, A_FREIO, MARGEM_M,
    PASSO_ENC, DT_FIS, DT_CTRL, N_SUB, X_LIM, _wrap,
)

# --- escalas próprias deste problema ---
PSID_ESC   = 15.0               # rad/s  (era 6,0; o swing-up chega a 13,5)
PSI_PRECISO = np.radians(10.0)  # dentro disto o bônus de precisão age

# --- pesos da recompensa (conferidos em _valores_de_referencia) ---
W_OMEGA  = 0.10     # nao girar sem parar
W_X2     = 0.20     # ficar perto do centro (quadrático)
W_X1     = 0.05     # gradiente de posição em todo x (linear)
W_DA     = 0.05     # suavidade; baixo de propósito, ver nota no final
W_BONUS  = 1.00     # bônus de precisão no topo
PSID_BONUS = 2.0    # rad/s: acima disto o bônus de precisão não vale nada
R_TRILHO = -50.0    # bater no limite de curso


class PenduloSwingUp:
    """
    Estado interno (verdadeiro):   x, v, phi, omega
    Observação (6):  [x/X_LIM, v/V_MAX, sin(psi), cos(psi), psiDot/PSID_ESC,
                      a_anterior/A_MAX]

    phi = 0 com a haste pendurada, pi com a haste em pé (convenção do firmware)
    psi = wrap(pi - phi)   -> 0 em pé, +-pi pendurado
    """

    dim_obs = 6
    dim_acao = 1

    def __init__(self, dominio_aleatorio=True, atraso_passos=1, semente=None,
                 ruido_encoder=True, max_passos=1200,
                 frac_topo=0.30, perturbacoes=True):
        self.rng = np.random.default_rng(semente)
        self.dominio_aleatorio = dominio_aleatorio
        self.atraso_passos = atraso_passos
        self.ruido_encoder = ruido_encoder
        self.max_passos = max_passos
        # frac_topo: fração dos episódios que começa PERTO DO TOPO em vez de
        # pendurada. Com 100% pendurado o crítico passa milhares de passos sem
        # ver recompensa alta nenhuma e a estimativa de valor não sai do zero;
        # com uma parte começando em pé, o sinal de "equilibrar vale muito"
        # existe desde a primeira iteração. É currículo, não trapaça: o agente
        # ainda precisa aprender a chegar lá sozinho nos outros 70%.
        self.frac_topo = frac_topo
        self.perturbacoes = perturbacoes

    # ---------------- randomização de domínio (idêntica à validada) --------
    def _sortear_mundo(self):
        r = self.rng
        if self.dominio_aleatorio:
            self.L   = L_EQ   * r.uniform(0.92, 1.08)
            self.b   = B_VISC * r.uniform(0.5, 1.8)
            self.c   = C_COUL * r.uniform(0.5, 1.8)
            self.ganho_a = r.uniform(0.90, 1.10)
            self.off_psi = r.normal(0, np.radians(0.8))
            self.atraso  = self.atraso_passos + r.integers(0, 2)
            self.escala_x = r.uniform(0.92, 1.08)
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
    def reset(self, psi0=None, x0=None, pendurado=None):
        self._sortear_mundo()
        r = self.rng

        if pendurado is None:
            pendurado = r.random() > self.frac_topo

        if psi0 is not None:
            psi = psi0
        elif pendurado:
            # haste pendurada e quase parada: é assim que a bancada começa,
            # depois do 'z' com o operador soltando a haste.
            psi = np.pi + r.uniform(-np.radians(20.0), np.radians(20.0))
            self.omega = r.uniform(-0.3, 0.3)
        else:
            psi = r.uniform(-np.radians(12.0), np.radians(12.0))
            self.omega = r.uniform(-0.8, 0.8)

        self.phi = np.pi - psi
        if psi0 is not None:
            self.omega = r.uniform(-0.3, 0.3)

        self.x = r.uniform(-0.10, 0.10) if x0 is None else x0
        self.v = 0.0
        self.a_cmd = 0.0
        self.fila = [0.0]*max(1, self.atraso)
        self.passos = 0
        self._reset_filtro()
        self._phi_q = self.phi
        return self._observar()

    def _medir(self):
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
            np.sin(psi),
            np.cos(psi),
            psid / PSID_ESC,
            self.a_cmd / A_MAX,
        ], dtype=np.float32)

    # ---------------- recompensa ----------------
    @staticmethod
    def recompensa(psi_v, psid_v, x, a_cmd, a_ant):
        """Separada e estática para poder ser conferida e testada sozinha."""
        altura = (1.0 + np.cos(psi_v)) / 2.0        # 0 pendurado, 1 em pé
        r = (altura
             - W_OMEGA*(psid_v/PSID_ESC)**2
             - W_X2*(abs(x)/X_LIM)**2
             - W_X1*(abs(x)/X_LIM)
             - W_DA*abs(a_cmd - a_ant)/A_MAX)
        if abs(psi_v) < PSI_PRECISO and abs(psid_v) < PSID_BONUS:
            # Sem este bônus, a diferença entre 8 graus e 0,5 grau é de
            # 0,0004 na altura -- indistinguível. O bônus é o que transforma
            # "esta em pe" em "esta em pe com precisao".
            #
            # A condição de |psiDot| NÃO é decorativa. Sem ela, um pêndulo
            # girando a 13 rad/s coleta o bônus inteiro ao passar pelo topo e
            # pontua 1,92 contra 2,00 do equilíbrio perfeito -- girar passa a
            # ser quase tão bom quanto equilibrar. Isto foi pego pela
            # verificação de valores ANTES de treinar, não depois de 4 horas.
            r += (W_BONUS
                  * (1.0 - abs(psi_v)/PSI_PRECISO)
                  * (1.0 - abs(psid_v)/PSID_BONUS))
        return r

    def step(self, acao):
        a_pedida = float(np.clip(np.asarray(acao).reshape(-1)[0], -1.0, 1.0)) * A_MAX
        a_ant = self.a_cmd
        self.a_cmd = a_pedida

        self.fila.append(a_pedida)
        a_efetiva = self.fila.pop(0) * self.ganho_a

        # Empurrão aleatório SÓ perto do topo. Durante o swing-up um chute
        # aleatório não ensina nada: ele só embaralha o bombeamento de energia
        # que o agente está tentando aprender. Perto do equilíbrio ele ensina
        # exactamente o que a etapa 8 mediu na bancada.
        psi_atual = _wrap(np.pi - self.phi)
        if (self.perturbacoes and abs(psi_atual) < np.radians(30.0)
                and self.rng.random() < 1/250):
            self.omega += self.rng.normal(0, 0.7)

        # ---- integra N_SUB passos de 1 ms, igual ao firmware ----
        for _ in range(N_SUB):
            v_bruta = self.v + a_efetiva*DT_FIS
            self.v = np.clip(v_bruta, -V_MAX, V_MAX)

            dB = max(0.0, (X_LIM_FIS - self.x) - MARGEM_M)
            dA = max(0.0, (self.x + X_LIM_FIS) - MARGEM_M)
            self.v = min(self.v, np.sqrt(2*A_FREIO*dB))
            self.v = max(self.v, -np.sqrt(2*A_FREIO*dA))

            a_real = (self.v - (v_bruta - a_efetiva*DT_FIS))/DT_FIS
            self.x += self.v*DT_FIS

            acc = ((-G*np.sin(self.phi) - a_real*np.cos(self.phi))/self.L
                   - self.b*self.omega
                   - self.c*np.tanh(self.omega/OM_EPS))
            self.omega += acc*DT_FIS
            self.phi   += self.omega*DT_FIS

            self._filtrar_omega(self._medir())

        self.passos += 1
        obs = self._observar()

        psi_v  = _wrap(np.pi - self.phi)      # valor VERDADEIRO
        psid_v = -self.omega

        # Só o trilho termina o episódio. O ângulo não termina mais nada:
        # a haste passando pelo fundo é parte do problema agora.
        caiu  = abs(self.x) > X_LIM
        trunc = self.passos >= self.max_passos

        r = R_TRILHO if caiu else self.recompensa(psi_v, psid_v, self.x,
                                                  self.a_cmd, a_ant)

        info = {'psi': psi_v, 'x': self.x, 'v': self.v, 'a': self.a_cmd,
                'altura': (1.0 + np.cos(psi_v))/2.0,
                'em_pe': abs(psi_v) < np.radians(10.0)}
        return obs, r, caiu, trunc, info

    def estado_verdadeiro(self):
        return np.array([self.x, self.v, _wrap(np.pi - self.phi), -self.omega])

    # ---------------- energia normalizada, como no firmware ----------------
    def energia(self):
        """En = 0.5*(omega/W0)^2 - cos(phi).  -1 pendurado parado, +1 no topo.

        É a mesma grandeza que o controlador de energia do firmware persegue
        com ALVO_EN = 1,10. Serve para DIAGNOSTICAR o que o agente aprendeu,
        não entra na observação nem na recompensa: se entrasse, estaríamos
        dando a solução clássica de graça.
        """
        W0 = np.sqrt(G/self.L)
        return 0.5*(self.omega/W0)**2 - np.cos(self.phi)


def _valores_de_referencia():
    """Confere que a recompensa premia o que queremos, ANTES de treinar.

    Uma recompensa mal pesada produz um agente que gira sem parar, e 4 horas
    de treino depois não há como saber se o problema foi o PPO ou o peso.
    """
    f = PenduloSwingUp.recompensa
    casos = [
        ('pendurado e parado',        np.pi,            0.0,  0.00, 0.0, 0.0),
        ('pendurado oscilando',       np.pi,            5.0,  0.00, 0.0, 0.0),
        ('na horizontal subindo',     np.pi/2,          8.0,  0.05, 4.0, 0.0),
        ('girando, passa pelo topo',  0.0,             13.2,  0.00, 0.0, 0.0),
        ('girando, no fundo',         np.pi,           13.2,  0.00, 0.0, 0.0),
        ('girando, na horizontal',    np.pi/2,         11.0,  0.00, 0.0, 0.0),
        ('em pe, 8 graus',            np.radians(8.0),  0.5,  0.02, 1.0, 0.5),
        ('em pe, 1,5 graus',          np.radians(1.5),  0.1,  0.02, 1.5, 0.0),
        ('em pe perfeito',            0.0,              0.0,  0.00, 0.0, 0.0),
    ]
    print('=== A RECOMPENSA PREMIA O QUE QUEREMOS? ===')
    print(f'{"estado":<28} {"altura":>7} {"recompensa":>11}')
    for nome, psi, psid, x, a, aa in casos:
        alt = (1.0 + np.cos(psi))/2.0
        print(f'{nome:<28} {alt:7.3f} {f(psi, psid, x, a, aa):11.3f}')
    print()
    print('A ordem tem de ser: em pe >> girando > pendurado.')
    print('Girando sem parar e a armadilha classica deste tipo de recompensa:')
    print('a altura MEDIA de um giro e 0,5, e sem a penalidade de |psiDot| o')
    print('agente descobre que girar paga mais que o risco de equilibrar.')


if __name__ == '__main__':
    _valores_de_referencia()
