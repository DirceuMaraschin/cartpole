"""Simula o fluxo placa->PC e confere se o laco de conferir_local.py
reproduz a cadeia de a_ant da placa EXATAMENTE.

A placa, por ciclo k:
    aRL = bruta[k-1] saturada e escalada
    bruta[k] = rede(obs(estado[k], aRL))
    envia pacote k, depois imprime "NET k bruta[k]"

Logo a linha NET k chega DEPOIS do pacote k -- e e esse atraso que a primeira
versao do script errou.
"""
import os
import sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from politica_c import PoliticaC
from rl_bridge import X_LIM, V_MAX, A_MAX, PSID_ESC_SW, ENC_SINAL, wrap

CPV = 2400
pol = PoliticaC()
rng = np.random.default_rng(7)
LB, PPM = 21403, 40000.0

def observar(est, a_ant):
    x = (est['pos'] - LB / 2) / PPM
    phi = ENC_SINAL * est['enc'] * (2 * np.pi / CPV)
    psi = wrap(np.pi - phi)
    return np.array([x / X_LIM, est['v'] / V_MAX, np.sin(psi), np.cos(psi),
                     -est['omega'] / PSID_ESC_SW, a_ant / A_MAX], np.float32)

# --- trajetoria plausivel: perto do topo, oscilando ---
N = 400
ests = []
for k in range(N):
    ests.append(dict(seq=k & 0xFF,
                     pos=int(LB/2 + 40 * np.sin(k/25)),
                     enc=int(1200 + 6 * np.sin(k/9)),
                     v=float(0.02*np.sin(k/13)),
                     omega=float(0.3*np.cos(k/7)),
                     flags=0x04 | 0x40))

# --- o que a placa faria ---
placa_ref, bruta_ant = {}, 0.0
for k, e in enumerate(ests):
    a_ant = float(np.clip(bruta_ant, -1, 1)) * A_MAX
    b = float(pol.agir(observar(e, a_ant))[0])
    placa_ref[k] = b
    bruta_ant = b

# --- fluxo que o PC ve: pacote k, e so depois NET k ---
class Fake:
    def __init__(self):
        self.net, self.k = {}, 0
    def ler_estado(self, prazo=0):
        if self.k >= N: return None
        if self.k > 0:                      # NET do pacote anterior chega agora
            self.net[(self.k-1) & 0xFF] = placa_ref[self.k-1]
        e = ests[self.k]; self.k += 1
        return e
    observar_sw = staticmethod(lambda e, a: (observar(e, a), 0.0, 0.0))

def rodar(corrigido):
    p = Fake()
    JANELA = 64
    pacotes, placa, pares = {}, {}, []
    bruta_ant = 0.0
    while True:
        est = p.ler_estado()
        if est is None: break
        if corrigido:
            pacotes[est['seq']] = est
            placa.update(p.net); p.net.clear()
            for seq in sorted(pacotes):
                ant = (seq - 1) & 0xFF
                if seq not in placa or ant not in placa: continue
                e = pacotes.pop(seq)
                a_ant = float(np.clip(placa[ant], -1, 1)) * A_MAX
                obs, _, _ = p.observar_sw(e, a_ant)
                pares.append((seq, float(pol.agir(obs)[0]), placa[seq]))
            while len(pacotes) > JANELA: pacotes.pop(next(iter(pacotes)))
            while len(placa) > JANELA: placa.pop(next(iter(placa)))
        else:   # a versao errada, para reproduzir os 82%
            a_ant = float(np.clip(bruta_ant, -1, 1)) * A_MAX
            obs, _, _ = p.observar_sw(est, a_ant)
            pacotes[est['seq']] = float(pol.agir(obs)[0])
            for seq in list(pacotes):
                if seq in p.net:
                    minha = pacotes.pop(seq); pl = p.net.pop(seq)
                    pares.append((seq, minha, pl)); bruta_ant = pl
    if not pares: return 0, 0.0
    e = np.array([abs(a-b) for _, a, b in pares])
    return len(pares), e.max()

for nome, corr in (('versao ERRADA (a de antes)', False),
                   ('versao CORRIGIDA', True)):
    n, mx = rodar(corr)
    print(f'{nome:<28} {n:4d} pares   erro max {mx:.3e}')
