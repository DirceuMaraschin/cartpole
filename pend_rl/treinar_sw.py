#!/usr/bin/env python3
"""
Treina um agente PPO para fazer TUDO: swing-up e equilíbrio, sem PID.

Uso:
    python treinar_sw.py                              # 2000 iteracoes
    python treinar_sw.py --iteracoes 500              # ensaio curto
    python treinar_sw.py --saida modelos_sw --nucleos 8

Salva em --saida:
    agente.pt       pesos finais
    melhor.pt       pesos da melhor avaliacao
    historico.csv   uma linha por iteracao

POR QUE A METRICA DE "MELHOR" MUDOU. No treino anterior o episodio MORRIA
quando |psi| passava de 35 graus, então "duração" media a qualidade: durar
1000 passos era sinônimo de equilibrar. Aqui o episódio só termina no trilho,
então a duração é quase sempre máxima e não distingue nada — um agente que
deixa a haste pendurada balançando dura o mesmo que um que equilibra.

A métrica passa a ser `frac_pe`: a fração da SEGUNDA METADE do episódio em que
|psi| < 10 graus. Ela responde às duas perguntas que importam de uma vez:
subiu? e ficou? Um agente que sobe e cai pontua baixo; um que gira sem parar
pontua baixo; só sobe e fica pontua alto.
"""

import argparse
import csv
import os
import time

import numpy as np
import torch

from pendulo_env_sw import PenduloSwingUp
from ppo import PPO, variancia_explicada

PE = np.radians(10.0)       # "em pé" para fins de medida


def avaliar(rede, n_ep=20, semente=12345, randomizar=True, max_passos=1200):
    """Avalia SEMPRE a partir da haste PENDURADA e sem empurrões.

    Pendurada porque é assim que a bancada começa: o operador solta a haste e
    o agente tem de resolver o resto. Avaliar a partir do topo mediria só o
    equilíbrio, que já sabemos fazer.

    Sem empurrões porque a avaliação precisa medir controle, não sorte: com
    chutes aleatórios ligados, a escolha do melhor modelo passa a depender de
    ruído.
    """
    env = PenduloSwingUp(dominio_aleatorio=randomizar, max_passos=max_passos,
                         semente=semente, perturbacoes=False, frac_topo=0.0)
    subiu, t_sub, frac_pe, psi_pe, xs, bateu = [], [], [], [], [], 0
    meio = max_passos // 2
    for _ in range(n_ep):
        obs = env.reset(pendurado=True)
        epe, ps, xx, prim = [], [], [], None
        for k in range(max_passos):
            acao, _, _ = rede.agir(obs, deterministico=True)
            obs, r, caiu, corte, info = env.step(acao)
            em_pe = abs(info['psi']) < PE
            epe.append(em_pe)
            xx.append(abs(info['x']))
            if em_pe:
                ps.append(abs(info['psi']))
                if prim is None:
                    prim = k
            if caiu:
                bateu += 1
                break
            if corte:
                break
        epe = np.array(epe)
        subiu.append(prim is not None)
        if prim is not None:
            t_sub.append(prim * 0.020)
        # fração da segunda metade em pé (se o episódio acabou cedo, conta zero
        # no que faltou: bater no trilho não pode sair de graça)
        seg = np.zeros(max_passos - meio, dtype=bool)
        n = min(len(epe) - meio, len(seg))
        if n > 0:
            seg[:n] = epe[meio:meio + n]
        frac_pe.append(seg.mean())
        if ps:
            psi_pe.append(np.degrees(np.mean(ps)))
        xs.append(np.mean(xx) * 1000)
    return {
        'frac_pe': float(np.mean(frac_pe) * 100),
        'subiu': float(np.mean(subiu) * 100),
        't_subida': float(np.mean(t_sub)) if t_sub else float('nan'),
        # psi_pe so existe se ALGUM episodio chegou em pe; nas primeiras
        # iteracoes nenhum chega, e np.nanmean([]) avisa em vez de devolver nan.
        'psi_pe': float(np.mean(psi_pe)) if psi_pe else float('nan'),
        'x': float(np.mean(xs)),
        'bateu': 100.0 * bateu / n_ep,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--iteracoes', type=int, default=2000)
    ap.add_argument('--passos', type=int, default=8192,
                    help='passos por lote; maior que no treino de equilibrio '
                         'porque os episodios sao mais longos')
    ap.add_argument('--epocas', type=int, default=10)
    ap.add_argument('--lote', type=int, default=256)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--gamma', type=float, default=0.995,
                    help='0.995 e nao 0.99: o swing-up leva ~175 passos e o '
                         'premio vem todo no fim; com 0.99 o horizonte efetivo '
                         'e 100 passos e o inicio do bombeamento fica cego')
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--clip', type=float, default=0.2)
    ap.add_argument('--entropia', type=float, default=0.01,
                    help='0.01 e nao 0.004: explorar mais, porque aqui ha uma '
                         'armadilha (girar sem parar) e um otimo distante')
    ap.add_argument('--kl-alvo', type=float, default=0.02)
    ap.add_argument('--semente', type=int, default=0)
    ap.add_argument('--saida', default='modelos_sw')
    ap.add_argument('--max-passos', type=int, default=1200)
    ap.add_argument('--frac-topo', type=float, default=0.30)
    ap.add_argument('--sem-randomizacao', action='store_true')
    ap.add_argument('--sem-perturbacoes', action='store_true')
    ap.add_argument('--aval-cada', type=int, default=25)
    ap.add_argument('--nucleos', type=int, default=0,
                    help='0 = deixa o torch decidir. Numa rede 2x64 um unico '
                         'thread costuma ser mais rapido')
    ap.add_argument('--retomar', default='',
                    help='caminho de um .pt para continuar de onde parou')
    args = ap.parse_args()

    if args.nucleos > 0:
        torch.set_num_threads(args.nucleos)

    os.makedirs(args.saida, exist_ok=True)
    randomizar = not args.sem_randomizacao

    env = PenduloSwingUp(dominio_aleatorio=randomizar,
                         max_passos=args.max_passos,
                         semente=args.semente,
                         perturbacoes=not args.sem_perturbacoes,
                         frac_topo=args.frac_topo)
    agente = PPO(dim_obs=env.dim_obs, dim_acao=env.dim_acao,
                 lr=args.lr, gamma=args.gamma, lam=args.lam,
                 eps_clip=args.clip, c_entropia=args.entropia,
                 epocas=args.epocas, tam_lote=args.lote,
                 n_passos=args.passos, kl_alvo=args.kl_alvo,
                 semente=args.semente)

    if args.retomar:
        est = torch.load(args.retomar, map_location='cpu', weights_only=True)
        agente.rede.load_state_dict(est['rede'])
        print(f'Retomando de {args.retomar}')

    tot = args.iteracoes * args.passos
    print(f'Treinando SWING-UP + EQUILIBRIO: {args.iteracoes} iteracoes '
          f'x {args.passos} = {tot/1e6:.1f} milhoes de passos')
    print(f'Observacao: 6 entradas (sin/cos do angulo) | '
          f'episodio {args.max_passos*0.020:.0f} s')
    print(f'gamma {args.gamma} | entropia {args.entropia} | '
          f'inicio no topo {100*args.frac_topo:.0f}% das vezes')
    print(f'Equivalente a {tot/50/3600:.0f} h de bancada -- '
          f'e por isso que se treina em simulacao.')
    print(f'Referencia sob esta recompensa: o PD classico faz 1,215 '
          f'por passo, partindo do topo.\n')

    arq = open(os.path.join(args.saida, 'historico.csv'), 'w', newline='')
    wr = csv.writer(arq)
    wr.writerow(['iteracao', 'passos', 'retorno', 'duracao_s', 'kl',
                 'clip_frac', 'entropia', 'std', 'var_explicada',
                 'frac_pe', 'subiu', 't_subida', 'psi_pe', 'x', 'bateu'])

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
                 '', '', '', '', '', '']

        if it % args.aval_cada == 0 or it == args.iteracoes:
            av = avaliar(agente.rede, n_ep=20, randomizar=randomizar,
                         max_passos=args.max_passos)
            linha[9:15] = [round(av['frac_pe'], 1), round(av['subiu'], 1),
                           round(av['t_subida'], 2), round(av['psi_pe'], 3),
                           round(av['x'], 1), round(av['bateu'], 1)]
            marca = ''
            if av['frac_pe'] > melhor:
                melhor = av['frac_pe']
                agente.salvar(os.path.join(args.saida, 'melhor.pt'))
                marca = '  <- salvo'
            print(f'it {it:5d} | ret {ret_med:8.1f} | KL {met["kl"]:.4f} | '
                  f'std {met["std"]:.3f} | VE {ve:5.2f} || '
                  f'subiu {av["subiu"]:5.1f}% em {av["t_subida"]:5.2f}s | '
                  f'em pe {av["frac_pe"]:5.1f}% | psi {av["psi_pe"]:5.2f}deg | '
                  f'trilho {av["bateu"]:4.0f}%{marca}')
        elif it % 10 == 0:
            print(f'it {it:5d} | ret {ret_med:8.1f} | dur {dur_med:5.2f}s | '
                  f'KL {met["kl"]:.4f} | std {met["std"]:.3f} | VE {ve:5.2f}')

        wr.writerow(linha)
        arq.flush()

    arq.close()
    agente.salvar(os.path.join(args.saida, 'agente.pt'))
    print(f'\nConcluido em {(time.time()-t0)/60:.1f} min')
    print(f'Pesos finais : {args.saida}/agente.pt')
    print(f'Melhor aval  : {args.saida}/melhor.pt')


if __name__ == '__main__':
    main()
