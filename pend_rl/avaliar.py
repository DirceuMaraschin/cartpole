#!/usr/bin/env python3
"""
Bateria de testes do agente treinado, antes de encostar no hardware.

Uso:
    python avaliar.py                             # todos os testes
    python avaliar.py --modelo modelos/melhor.pt
    python avaliar.py --teste robustez
    python avaliar.py --graficos                  # gera as figuras

Testes:
    1. desempenho   -- sucesso, oscilacao, centragem, suavidade
    2. robustez     -- varre cada perturbacao isoladamente
    3. comparacao   -- agente x PID do firmware, lado a lado
    4. perturbacao  -- empurroes no meio do episodio
    5. estresse     -- angulos e posicoes iniciais fora do normal
"""

import argparse, os
import numpy as np

from pendulo_env import PenduloInvertido, PIDFirmware, PSI_LIM, X_LIM, A_MAX
import pendulo_env as E
from ppo import AtorCritico
import torch


# ---------------------------------------------------------------- utilidades
def mk_env(**kw):
    """Cria o ambiente SEM empurrões aleatórios.

    Os empurrões pertencem ao TREINAMENTO (ensinam recuperação) e ao teste 4,
    que os mede de propósito. Deixá-los ligados nos demais testes misturaria
    'o agente controla bem?' com 'o agente aguenta empurrão?', e nenhuma das
    duas respostas sairia limpa.
    """
    kw.setdefault('perturbacoes', False)
    kw.setdefault('max_passos', 1000)
    return PenduloInvertido(**kw)


def carregar(caminho):
    rede = AtorCritico(dim_obs=5, dim_acao=1)
    d = torch.load(caminho, map_location='cpu', weights_only=True)
    rede.load_state_dict(d['rede'])
    rede.eval()
    return rede


def politica_agente(rede):
    def f(obs):
        a, _, _ = rede.agir(obs, deterministico=True)
        return a
    return f


def politica_pid():
    pid = PIDFirmware()
    def f(obs):
        return np.array([pid(obs)], dtype=np.float32)
    f.reset = pid.reset
    return f


def episodio(env, politica, obs=None, empurrao=None):
    """Roda um episodio. empurrao = (passo, delta_omega) para perturbar."""
    obs = env.reset() if obs is None else obs
    if hasattr(politica, 'reset'):
        politica.reset()
    ps, xs, das = [], [], []
    a_ant = 0.0
    for k in range(env.max_passos):
        if empurrao and k == empurrao[0]:
            env.omega += empurrao[1]
        acao = politica(obs)
        obs, r, caiu, corte, info = env.step(acao)
        ps.append(abs(info['psi'])); xs.append(abs(info['x']))
        das.append(abs(info['a'] - a_ant)); a_ant = info['a']
        if caiu or corte:
            break
    return {
        'passos': k + 1,
        'duracao': (k + 1) * 0.020,
        'caiu': caiu,
        'psi': np.degrees(np.mean(ps)),
        'psi_max': np.degrees(np.max(ps)),
        'x': np.mean(xs) * 1000,
        'x_max': np.max(xs) * 1000,
        'suav': np.mean(das),
    }


def lote(env, politica, n=30, semente=999):
    env.rng = np.random.default_rng(semente)
    rs = [episodio(env, politica) for _ in range(n)]
    dur = np.array([r['duracao'] for r in rs])
    alvo = env.max_passos * 0.020
    return {
        'sucesso': float(np.mean(dur >= alvo - 1e-9) * 100),
        'duracao': float(dur.mean()),
        'psi': float(np.mean([r['psi'] for r in rs])),
        'psi_max': float(np.mean([r['psi_max'] for r in rs])),
        'x': float(np.mean([r['x'] for r in rs])),
        'x_max': float(np.mean([r['x_max'] for r in rs])),
        'suav': float(np.mean([r['suav'] for r in rs])),
    }


def cab():
    print(f'{"":28s} {"sucesso":>8} {"dur":>7} {"|psi|":>8} {"psi max":>8} '
          f'{"|x|":>7} {"x max":>7} {"suavid":>8}')


def linha(nome, m):
    print(f'{nome:28s} {m["sucesso"]:7.1f}% {m["duracao"]:6.2f}s '
          f'{m["psi"]:7.2f}d {m["psi_max"]:7.2f}d {m["x"]:6.0f}mm '
          f'{m["x_max"]:6.0f}mm {m["suav"]:7.3f}')


# ---------------------------------------------------------------- testes
def teste_desempenho(rede):
    print('\n=== TESTE 1: DESEMPENHO ===')
    print('30 episodios de 20 s, politica deterministica\n')
    cab()
    for nome, rand in [('mundo nominal', False), ('com randomizacao', True)]:
        env = mk_env(dominio_aleatorio=rand, max_passos=1000)
        linha(nome, lote(env, politica_agente(rede)))
    print('\nmeta: sucesso > 85% com randomizacao, |psi| < 2 graus, |x| < 80 mm')


def teste_robustez(rede):
    print('\n=== TESTE 2: ROBUSTEZ (cada perturbacao isolada) ===')
    print('Mostra o que derruba o agente. 30 episodios cada.\n')

    class Env1(PenduloInvertido):
        alvo = None
        intens = 1.0
        def _sortear_mundo(self):
            r = self.rng
            self.L, self.b, self.c = E.L_EQ, E.B_VISC, E.C_COUL
            self.ganho_a = 1.0; self.off_psi = 0.0
            self.atraso = 1; self.escala_x = 1.0
            a, s = self.alvo, self.intens
            if a == 'L':        self.L = E.L_EQ * r.uniform(1-0.08*s, 1+0.08*s)
            elif a == 'atrito': self.b = E.B_VISC*r.uniform(0.5,1.8); self.c = E.C_COUL*r.uniform(0.5,1.8)
            elif a == 'ganho':  self.ganho_a = r.uniform(1-0.10*s, 1+0.10*s)
            elif a == 'offset': self.off_psi = r.normal(0, np.radians(0.8*s))
            elif a == 'atraso': self.atraso = 1 + int(r.integers(0, 2))
            elif a == 'atraso2': self.atraso = 2 + int(r.integers(0, 2))
            elif a == 'escala': self.escala_x = r.uniform(1-0.08*s, 1+0.08*s)

    cab()
    for alvo, rot in [(None,'nenhuma (referencia)'), ('L','comprimento +-8%'),
                      ('atrito','atrito x0,5 a x1,8'), ('ganho','ganho de acel. +-10%'),
                      ('offset','offset de zero 0,8 deg'), ('escala','escala de x +-8%'),
                      ('atraso','atraso 1-2 passos'), ('atraso2','atraso 2-3 passos (extra)')]:
        Env1.alvo = alvo
        env = Env1(max_passos=1000, perturbacoes=False)
        linha(rot, lote(env, politica_agente(rede)))

    print('\n--- offset de zero: ate onde aguenta ---')
    cab()
    Env1.alvo = 'offset'
    for s in [1.0, 2.0, 3.0, 4.0]:
        Env1.intens = s
        env = Env1(max_passos=1000, perturbacoes=False)
        linha(f'offset sigma = {0.8*s:.1f} deg', lote(env, politica_agente(rede)))
    Env1.intens = 1.0


def teste_comparacao(rede):
    print('\n=== TESTE 3: AGENTE x PID ===')
    print('Mesmos episodios, mesmas sementes.\n')
    cab()
    for rand, rot in [(False, 'nominal'), (True, 'randomizado')]:
        env = mk_env(dominio_aleatorio=rand, max_passos=1000)
        linha(f'AGENTE  ({rot})', lote(env, politica_agente(rede)))
        env = mk_env(dominio_aleatorio=rand, max_passos=1000)
        linha(f'PID firmware ({rot})', lote(env, politica_pid()))
    print('\nreferencia da bancada real: PID a 1 kHz com |psi| = 1,90 deg,')
    print('mas derivando ate 175 mm do centro.')


def teste_perturbacao(rede):
    print('\n=== TESTE 4: EMPURROES ===')
    print('Chute de velocidade angular no meio do episodio.')
    print('Simula o dedo batendo na haste.\n')
    print(f'{"empurrao":>18} {"agente":>12} {"PID":>12}')
    for dom in [1.0, 2.0, 3.0, 4.0, 5.0]:
        res = []
        for pol_f in [lambda: politica_agente(rede), politica_pid]:
            env = mk_env(dominio_aleatorio=False, max_passos=1000)
            env.rng = np.random.default_rng(555)
            ok = 0
            for _ in range(20):
                r = episodio(env, pol_f(), empurrao=(250, dom))
                ok += (not r['caiu'])
            res.append(100.0 * ok / 20)
        print(f'{dom:14.1f} rad/s {res[0]:10.0f}% {res[1]:11.0f}%')


def teste_estresse(rede):
    print('\n=== TESTE 5: CONDICOES INICIAIS EXTREMAS ===')
    print('De que angulo e posicao o agente ainda recupera?\n')
    env = mk_env(dominio_aleatorio=False, max_passos=1000)
    pol = politica_agente(rede)
    print(f'{"psi inicial":>12} {"x inicial":>11} {"recupera":>10}')
    for psi0 in [3, 6, 10, 15, 20, 25, 30]:
        for x0 in [0.0, 0.12]:
            ok = 0
            for s in range(10):
                env.rng = np.random.default_rng(100 + s)
                obs = env.reset(psi0=np.radians(psi0) * (1 if s % 2 else -1), x0=x0)
                r = episodio(env, pol, obs=obs)
                ok += (not r['caiu'])
            print(f'{psi0:10d}deg {x0*1000:8.0f}mm {100*ok/10:8.0f}%')


def graficos(rede, saida='modelos'):
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt, pandas as pd
    plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': .3, 'figure.dpi': 130})

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    h = pd.read_csv(os.path.join(saida, 'historico.csv'))
    av = h.dropna(subset=['aval_sucesso'])

    ax[0,0].plot(h.passos/1000, h.retorno, lw=.8, color='#9fb0c2')
    ax[0,0].plot(h.passos/1000, h.retorno.rolling(10, min_periods=1).mean(), lw=2, color='#1f3f66')
    ax[0,0].set_title('Retorno por episodio', loc='left', fontweight='bold')
    ax[0,0].set_xlabel('mil passos'); ax[0,0].set_ylabel('retorno')

    ax[0,1].plot(av.passos/1000, av.aval_sucesso, 'o-', color='#2f6b4f', ms=4)
    ax[0,1].axhline(85, ls='--', color='#a33a3a', lw=1.2)
    ax[0,1].annotate('meta 85%', xy=(av.passos.iloc[0]/1000, 87), fontsize=8, color='#a33a3a')
    ax[0,1].set_title('Taxa de sucesso na avaliacao', loc='left', fontweight='bold')
    ax[0,1].set_xlabel('mil passos'); ax[0,1].set_ylabel('%'); ax[0,1].set_ylim(-5, 105)

    ax[1,0].plot(h.passos/1000, h.kl, lw=1, color='#b5651d', label='KL')
    ax[1,0].axhline(.02, ls='--', color='#a33a3a', lw=1)
    ax[1,0].set_ylabel('KL', color='#b5651d')
    a2 = ax[1,0].twinx()
    a2.plot(h.passos/1000, h.clip_frac, lw=1, color='#2f6ba8')
    a2.set_ylabel('fracao recortada', color='#2f6ba8'); a2.grid(False)
    ax[1,0].set_title('Saude do passo de otimizacao', loc='left', fontweight='bold')
    ax[1,0].set_xlabel('mil passos')

    ax[1,1].plot(h.passos/1000, h.var_explicada, lw=1.4, color='#5b4a9e', label='variancia explicada')
    ax[1,1].plot(h.passos/1000, h['std'], lw=1.4, color='#8a6d1f', label='desvio da politica')
    ax[1,1].set_title('Critico e exploracao', loc='left', fontweight='bold')
    ax[1,1].set_xlabel('mil passos'); ax[1,1].legend(); ax[1,1].set_ylim(-.3, 1.3)

    plt.tight_layout(); plt.savefig(os.path.join(saida, 'treinamento.png'), bbox_inches='tight')
    print(f'salvo {saida}/treinamento.png')

    # trajetoria de um episodio
    env = mk_env(dominio_aleatorio=True, max_passos=1000, semente=42)
    obs = env.reset(); T=[]
    for k in range(1000):
        a, _, _ = rede.agir(obs, deterministico=True)
        obs, r, c, t, i = env.step(a)
        T.append([k*0.02, np.degrees(i['psi']), i['x']*1000, i['v'], i['a']])
        if c or t: break
    T = np.array(T)
    fig, ax = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    for k, (col, rot, cor) in enumerate([(1,'angulo psi (graus)','#1f3f66'),
                                         (2,'posicao x (mm)','#2f6b4f'),
                                         (3,'velocidade (m/s)','#b5651d'),
                                         (4,'acao: aceleracao (m/s2)','#5b4a9e')]):
        ax[k].plot(T[:,0], T[:,col], lw=1.2, color=cor); ax[k].set_ylabel(rot, fontsize=8)
    ax[0].axhline(35, ls='--', color='#a33a3a', lw=1); ax[0].axhline(-35, ls='--', color='#a33a3a', lw=1)
    ax[1].axhline(240, ls='--', color='#a33a3a', lw=1); ax[1].axhline(-240, ls='--', color='#a33a3a', lw=1)
    ax[0].set_title(f'Um episodio completo ({T[-1,0]:.1f} s)', loc='left', fontweight='bold')
    ax[3].set_xlabel('tempo (s)')
    plt.tight_layout(); plt.savefig(os.path.join(saida, 'episodio.png'), bbox_inches='tight')
    print(f'salvo {saida}/episodio.png')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--modelo', default='modelos/melhor.pt')
    ap.add_argument('--teste', default='todos',
                    choices=['todos','desempenho','robustez','comparacao','perturbacao','estresse'])
    ap.add_argument('--graficos', action='store_true')
    args = ap.parse_args()

    rede = carregar(args.modelo)
    print(f'Modelo: {args.modelo}')

    t = args.teste
    if t in ('todos','desempenho'):  teste_desempenho(rede)
    if t in ('todos','robustez'):    teste_robustez(rede)
    if t in ('todos','comparacao'):  teste_comparacao(rede)
    if t in ('todos','perturbacao'): teste_perturbacao(rede)
    if t in ('todos','estresse'):    teste_estresse(rede)
    if args.graficos:                graficos(rede, os.path.dirname(args.modelo) or '.')


if __name__ == '__main__':
    main()
