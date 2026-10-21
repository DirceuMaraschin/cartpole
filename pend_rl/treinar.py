#!/usr/bin/env python3
"""
Treina o agente PPO no simulador do pêndulo invertido.

Uso típico:
    python treinar.py                          # treino padrão, ~300 iteracoes
    python treinar.py --iteracoes 150          # mais curto
    python treinar.py --sem-randomizacao       # mundo fixo (so para diagnostico)
    python treinar.py --semente 7 --saida modelos/run7

Salva em modelos/:
    agente.pt        pesos finais
    melhor.pt        pesos da melhor avaliacao
    historico.csv    uma linha por iteracao, para os graficos
"""

import argparse, csv, os, time
import numpy as np

from pendulo_env import PenduloInvertido
from ppo import PPO, variancia_explicada


def avaliar(rede, n_ep=20, semente=12345, randomizar=True, max_passos=1000):
    """Mede o desempenho com política DETERMINÍSTICA (sem exploração).

    Sempre SEM empurrões: a avaliação precisa medir a qualidade do controle,
    não a sorte nos chutes aleatórios. Se os empurrões ficassem ligados, a
    escolha do melhor modelo passaria a depender de ruído.
    """
    env = PenduloInvertido(dominio_aleatorio=randomizar, max_passos=max_passos,
                           semente=semente, perturbacoes=False)
    duracoes, psis, xs, dacoes = [], [], [], []
    for _ in range(n_ep):
        obs = env.reset()
        ps, xx, da = [], [], []
        a_ant = 0.0
        for k in range(max_passos):
            acao, _, _ = rede.agir(obs, deterministico=True)
            obs, r, caiu, corte, info = env.step(acao)
            ps.append(abs(info['psi'])); xx.append(abs(info['x']))
            da.append(abs(info['a'] - a_ant)); a_ant = info['a']
            if caiu or corte:
                break
        duracoes.append((k + 1) * 0.020)
        psis.append(np.degrees(np.mean(ps)))
        xs.append(np.mean(xx) * 1000)
        dacoes.append(np.mean(da))
    duracoes = np.array(duracoes)
    return {
        'sucesso': float(np.mean(duracoes >= max_passos * 0.020 - 1e-6) * 100),
        'duracao': float(duracoes.mean()),
        'psi': float(np.mean(psis)),
        'x': float(np.mean(xs)),
        'suavidade': float(np.mean(dacoes)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--iteracoes', type=int, default=300)
    ap.add_argument('--passos', type=int, default=4096, help='passos por lote')
    ap.add_argument('--epocas', type=int, default=10)
    ap.add_argument('--lote', type=int, default=64)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--gamma', type=float, default=0.99)
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--clip', type=float, default=0.2)
    ap.add_argument('--entropia', type=float, default=0.004)
    ap.add_argument('--kl-alvo', type=float, default=0.02)
    ap.add_argument('--semente', type=int, default=0)
    ap.add_argument('--saida', default='modelos')
    ap.add_argument('--sem-randomizacao', action='store_true')
    ap.add_argument('--sem-perturbacoes', action='store_true',
                    help='desliga os empurroes aleatorios durante o episodio')
    ap.add_argument('--inicio-estreito', action='store_true',
                    help='estado inicial sempre perto do equilibrio (NAO recomendado)')
    ap.add_argument('--aval-cada', type=int, default=10)
    args = ap.parse_args()

    os.makedirs(args.saida, exist_ok=True)
    randomizar = not args.sem_randomizacao

    env = PenduloInvertido(dominio_aleatorio=randomizar, max_passos=1000,
                           semente=args.semente,
                           perturbacoes=not args.sem_perturbacoes,
                           inicio_amplo=not args.inicio_estreito)
    agente = PPO(dim_obs=env.dim_obs, dim_acao=env.dim_acao,
                 lr=args.lr, gamma=args.gamma, lam=args.lam,
                 eps_clip=args.clip, c_entropia=args.entropia,
                 epocas=args.epocas, tam_lote=args.lote,
                 n_passos=args.passos, kl_alvo=args.kl_alvo,
                 semente=args.semente)

    print(f'Treinando: {args.iteracoes} iteracoes x {args.passos} passos '
          f'= {args.iteracoes*args.passos/1000:.0f} mil passos')
    print(f'Randomizacao de dominio: {"LIGADA" if randomizar else "desligada"}')
    print(f'Inicio amplo: {"desligado" if args.inicio_estreito else "LIGADO"} | '
          f'Empurroes: {"desligados" if args.sem_perturbacoes else "LIGADOS"}')
    print(f'Cada 1000 passos simulados = 20 s de bancada\n')

    arq = open(os.path.join(args.saida, 'historico.csv'), 'w', newline='')
    wr = csv.writer(arq)
    wr.writerow(['iteracao', 'passos', 'retorno', 'duracao_s', 'kl', 'clip_frac',
                 'entropia', 'std', 'var_explicada', 'aval_sucesso', 'aval_psi',
                 'aval_x', 'aval_duracao'])

    obs = env.reset()
    melhor = -1.0
    t0 = time.time()

    for it in range(1, args.iteracoes + 1):
        buf, obs, retornos, duracoes = agente.coletar(env, obs)
        met = agente.atualizar(buf)
        ve = variancia_explicada(buf.val, buf.ret)

        ret_med = float(np.mean(retornos)) if retornos else float('nan')
        dur_med = float(np.mean(duracoes)) * 0.020 if duracoes else float('nan')

        linha = [it, it * args.passos, round(ret_med, 2), round(dur_med, 3),
                 round(met['kl'], 5), round(met['clip_frac'], 4),
                 round(met['entropia'], 4), round(met['std'], 4), round(ve, 4),
                 '', '', '', '']

        if it % args.aval_cada == 0 or it == args.iteracoes:
            av = avaliar(agente.rede, n_ep=20, randomizar=randomizar)
            linha[9:13] = [round(av['sucesso'], 1), round(av['psi'], 3),
                           round(av['x'], 1), round(av['duracao'], 2)]
            marca = ''
            if av['sucesso'] > melhor or (av['sucesso'] == melhor and av['psi'] < 99):
                melhor = av['sucesso']
                agente.salvar(os.path.join(args.saida, 'melhor.pt'))
                marca = '  <- salvo'
            print(f'it {it:4d} | ret {ret_med:7.1f} | dur {dur_med:5.2f}s | '
                  f'KL {met["kl"]:.4f} | clip {met["clip_frac"]:.2f} | '
                  f'std {met["std"]:.3f} | VE {ve:5.2f} || '
                  f'AVAL sucesso {av["sucesso"]:5.1f}% psi {av["psi"]:5.2f}deg '
                  f'x {av["x"]:5.1f}mm{marca}')
        elif it % 5 == 0:
            print(f'it {it:4d} | ret {ret_med:7.1f} | dur {dur_med:5.2f}s | '
                  f'KL {met["kl"]:.4f} | clip {met["clip_frac"]:.2f} | '
                  f'std {met["std"]:.3f} | VE {ve:5.2f} | ep {met["epocas_usadas"]}')

        wr.writerow(linha); arq.flush()

    arq.close()
    agente.salvar(os.path.join(args.saida, 'agente.pt'))
    print(f'\nConcluido em {(time.time()-t0)/60:.1f} min')
    print(f'Pesos finais : {args.saida}/agente.pt')
    print(f'Melhor aval  : {args.saida}/melhor.pt')


if __name__ == '__main__':
    main()
