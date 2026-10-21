#!/usr/bin/env python3
"""Compara os agentes treinados e o PID na mesma bateria."""
import numpy as np
from pendulo_env import PenduloInvertido
from avaliar import carregar, politica_agente, politica_pid, lote, episodio, mk_env

MODELOS = [('v1 inicio estreito, sem empurroes', 'modelos_v1/melhor.pt'),
           ('v2 inicio amplo + empurroes',       'modelos_v2/agente.pt'),
           ('v3 inicio amplo, sem empurroes',    'modelos/melhor.pt')]


def recuperacao(pol_f, ang, n=20):
    env = mk_env(dominio_aleatorio=False)
    ok = 0
    for s in range(n):
        env.rng = np.random.default_rng(300 + s)
        obs = env.reset(psi0=np.radians(ang) * (1 if s % 2 else -1), x0=0.0)
        ok += (not episodio(env, pol_f(), obs=obs)['caiu'])
    return 100.0 * ok / n


def empurrao(pol_f, dom=1.0, n=20):
    env = mk_env(dominio_aleatorio=False)
    env.rng = np.random.default_rng(555)
    ok = 0
    for _ in range(n):
        ok += (not episodio(env, pol_f(), empurrao=(250, dom))['caiu'])
    return 100.0 * ok / n


def main():
    print('=== COMPARACAO ===')
    print('sucesso/psi/x: 30 episodios de 20 s com randomizacao de dominio')
    print('rec Xd: recupera de angulo inicial de X graus | empurrao: chute de 1,0 rad/s\n')
    print(f'{"":34s} {"suc":>5} {"|psi|":>7} {"|x|":>6} {"suavid":>7} '
          f'{"rec 10d":>8} {"rec 20d":>8} {"empurrao":>9}')

    for nome, cam in MODELOS:
        try:
            rede = carregar(cam)
        except Exception as e:
            print(f'{nome:34s} (nao encontrado: {e})')
            continue
        pol_f = lambda r=rede: politica_agente(r)
        m = lote(mk_env(dominio_aleatorio=True), politica_agente(rede), n=30)
        print(f'{nome:34s} {m["sucesso"]:4.0f}% {m["psi"]:6.2f}d {m["x"]:5.0f}mm '
              f'{m["suav"]:7.3f} {recuperacao(pol_f,10):7.0f}% '
              f'{recuperacao(pol_f,20):7.0f}% {empurrao(pol_f):8.0f}%')

    m = lote(mk_env(dominio_aleatorio=True), politica_pid(), n=30)
    print(f'{"PID do firmware":34s} {m["sucesso"]:4.0f}% {m["psi"]:6.2f}d {m["x"]:5.0f}mm '
          f'{m["suav"]:7.3f} {recuperacao(politica_pid,10):7.0f}% '
          f'{recuperacao(politica_pid,20):7.0f}% {empurrao(politica_pid):8.0f}%')


if __name__ == '__main__':
    main()
