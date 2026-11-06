#!/usr/bin/env python3
"""
Compara torch, C exato e C com LUT -- em MALHA FECHADA.

É o teste que decide a LUT de tanh. O erro pontual de 5,8e-03 m/s² medido em
vetores estáticos não responde a pergunta certa: num sistema cuja constante de
tempo é 0,151 s e que dobra o erro a cada 105 ms, o que importa é se o agente
ainda sobe e ainda fica.

DUAS BATERIAS, nesta ordem de propósito:

  A. SOMBRA -- o torch dirige, as três políticas opinam no mesmo estado.
     Mede divergência em estados que a trajetória realmente visita, não em
     amostras sintéticas. Se falhar aqui, não vale rodar a bateria B.

  B. PAREADA -- cada política dirige seus próprios episódios, com a MESMA
     semente. Como `perturbacoes=False`, o RNG do ambiente é consumido só no
     reset: episódio k parte do mesmo estado para as três. Comparação pareada
     de verdade, sem precisar de mais sementes para afogar o ruído.

O CRITÉRIO NÃO PODE SER UM LIMIAR FIXO. A primeira versão deste arquivo
reprovava diferença de 1 ponto percentual em `frac_pe` -- e reprovou o C com
`tanhf`, cujo erro pontual é 1,4e-06, puro arredondamento de float32.

O que delatou o critério: o C exato e o C com LUT deram o MESMO desvio, apesar
de erros 1000x diferentes. Se a causa fosse magnitude de erro, a LUT seria
pior. A causa é outra: a planta é instável, o erro dobra a cada 105 ms, e um
desvio de 1e-06 decorrelaciona a trajetória em ~2 s. Depois disso os episódios
são outros, e `frac_pe` difere por amostragem.

Então a bateria B compara contra um PISO DE RUÍDO medido: um braço de controle
que é o próprio torch com um tremor de 1e-06 na saída -- uma perturbação que
sabemos inofensiva, da mesma ordem da que o C introduz. Uma variante em C é
aprovada se não se afasta mais do que esse braço. A estatística é a diferença
PAREADA por episódio (os episódios partem do mesmo estado), com erro padrão:
é ele que diz se uma diferença de média significa algo.

Uso:
    python comparar_c.py
    python comparar_c.py --n 50 --sem-lut
"""

import argparse
import os
import sys
import time

import numpy as np
import torch

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
torch.set_num_threads(1)

from pendulo_env_sw import PenduloSwingUp
from avaliar_sw import rodar, media, carregar
from politica_c import PoliticaC

PE = np.radians(10.0)
A_MAX = 8.0


# ======================================================================
# A. sombra
# ======================================================================
def sombra(pols, n_ep=20, passos=1200, semente=4242):
    """O primeiro da lista dirige; todos opinam no mesmo estado."""
    nomes = list(pols)
    lider = nomes[0]
    env = PenduloSwingUp(max_passos=passos, semente=semente,
                         perturbacoes=False, frac_topo=0.0)

    dif = {n: [] for n in nomes[1:]}        # erro na ACELERAÇÃO, m/s2
    dif_cru = {n: [] for n in nomes[1:]}
    n_passos = 0

    for _ in range(n_ep):
        obs = env.reset(pendurado=True)
        for _ in range(passos):
            a_lider = pols[lider](obs)
            base = float(np.clip(np.asarray(a_lider).reshape(-1)[0], -1, 1))
            for n in nomes[1:]:
                a = float(np.asarray(pols[n](obs)).reshape(-1)[0])
                dif_cru[n].append(abs(a - float(np.asarray(a_lider).reshape(-1)[0])))
                dif[n].append(abs(np.clip(a, -1, 1) - base) * A_MAX)
            n_passos += 1
            obs, _, caiu, corte, _ = env.step(a_lider)
            if caiu or corte:
                break

    print(f'--- A. SOMBRA: {lider} dirige, {n_passos} estados visitados ---')
    print(f'{"politica":<20} {"erro cru max":>14} {"erro m/s2 max":>14} '
          f'{"m/s2 medio":>12}')
    ok = True
    for n in nomes[1:]:
        c = np.array(dif_cru[n]); a = np.array(dif[n])
        print(f'{n:<20} {c.max():14.3e} {a.max():14.3e} {a.mean():12.3e}')
        if a.max() > 0.08:                  # 1% de A_MAX
            ok = False
    if not ok:
        print('  >> ATENCAO: divergencia acima de 1% de A_MAX em algum estado '
              'da trajetoria.')
    return ok


# ======================================================================
# B. pareada
# ======================================================================
def _ep(pol, n_ep, passos, semente):
    """Roda n_ep episódios e devolve as séries POR EPISÓDIO, para parear."""
    env = PenduloSwingUp(max_passos=passos, semente=semente,
                         perturbacoes=False, frac_topo=0.0)
    t0 = time.perf_counter()
    rs = [rodar(pol, env) for _ in range(n_ep)]
    return rs, time.perf_counter() - t0


def _dif_pareada(a, b):
    """Média e erro padrão da diferença pareada, ignorando NaN em qualquer lado."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    d = a[m] - b[m]
    if d.size < 2:
        return float('nan'), float('nan')
    return float(d.mean()), float(d.std(ddof=1) / np.sqrt(d.size))


def pareada(pols, n_ep=30, passos=1200, semente=4242):
    print(f'\n--- B. PAREADA: {n_ep} episodios de {passos*0.020:.0f} s, '
          f'mesma semente para todas ---')
    print(f'{"politica":<20} {"sobe":>6} {"t sub":>7} {"em pe":>7} '
          f'{"psi":>7} {"<|x|>":>7} {"trilho":>7} {"seg":>6}')

    bruto = {}
    for nome, pol in pols.items():
        rs, dt = _ep(pol, n_ep, passos, semente)
        bruto[nome] = rs
        print(f'{nome:<20} {100*np.mean([r["subiu"] for r in rs]):5.0f}% '
              f'{media(rs,"t_subida"):6.2f}s {100*media(rs,"frac_pe"):6.1f}% '
              f'{media(rs,"psi_pe"):6.2f}d {media(rs,"x_med"):6.0f}mm '
              f'{100*np.mean([r["caiu"] for r in rs]):6.0f}% {dt:5.1f}s')

    ref_nome = next(iter(pols))
    ref = bruto[ref_nome]

    def serie(rs, k, esc=1.0):
        return [r[k] * esc for r in rs]

    print(f'\n--- diferenca PAREADA vs {ref_nome} (media +- erro padrao) ---')
    print(f'{"politica":<20} {"em pe (pp)":>20} {"psi (graus)":>20}')

    dif = {}
    for nome, rs in bruto.items():
        if nome == ref_nome:
            continue
        dfp, sfp = _dif_pareada(serie(rs, 'frac_pe', 100),
                                serie(ref, 'frac_pe', 100))
        dps, sps = _dif_pareada(serie(rs, 'psi_pe'), serie(ref, 'psi_pe'))
        dif[nome] = (dfp, sfp, dps, sps)
        print(f'{nome:<20} {dfp:+11.2f} +-{sfp:5.2f} '
              f'{dps:+11.3f} +-{sps:5.3f}')

    # --- veredicto: nenhuma variante em C pode se afastar mais que o braço
    #     de tremor, que é uma perturbação sabidamente inofensiva ---
    piso = next((n for n in dif if 'tremor' in n), None)
    if piso is None:
        print('\n(sem braco de tremor: nao da para julgar significancia)')
        return True

    pfp, pfs, pps, pss = dif[piso]
    lim_fp = abs(pfp) + 2 * max(pfs, 1e-9)
    lim_ps = abs(pps) + 2 * max(pss, 1e-9)
    print(f'\nPiso de ruido ({piso}): em pe {abs(pfp):.2f}pp, '
          f'psi {abs(pps):.3f}d')
    print(f'Limite aceitavel:       em pe {lim_fp:.2f}pp, psi {lim_ps:.3f}d')

    veredicto = True
    for nome, (dfp, sfp, dps, sps) in dif.items():
        if nome == piso:
            continue
        mau = abs(dfp) > lim_fp or abs(dps) > lim_ps
        print(f'  {nome:<18} {"REPROVOU" if mau else "ok"}')
        if mau:
            veredicto = False
    return veredicto


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--modelo', default='modelos_sw/melhor.pt')
    ap.add_argument('--n', type=int, default=30, help='episodios da bateria B')
    ap.add_argument('--n-sombra', type=int, default=20)
    ap.add_argument('--passos', type=int, default=1200)
    ap.add_argument('--semente', type=int, default=4242)
    ap.add_argument('--sem-lut', action='store_true')
    args = ap.parse_args()

    if not os.path.exists(args.modelo):
        raise SystemExit(f'nao achei {args.modelo}')

    print(f'Modelo {args.modelo}\n')

    t_ref = carregar(args.modelo)
    pols = {'torch (referencia)': t_ref}

    # Braço de controle: o MESMO torch com 1e-06 de tremor na saída. Nenhum
    # controlador real se degrada com isso; serve para medir quanto a metrica
    # se move sob uma perturbacao da ordem da que o float32 introduz.
    rng_t = np.random.default_rng(99)
    pols['torch + tremor'] = lambda o: (np.asarray(t_ref(o), dtype=np.float64)
                                        + rng_t.normal(0.0, 1e-6))

    c_ex = PoliticaC(lut=False)
    pols['C + tanhf'] = lambda o, p=c_ex: p.agir(o)
    if not args.sem_lut:
        c_lut = PoliticaC(lut=True)
        pols['C + LUT'] = lambda o, p=c_lut: p.agir(o)

    ok_a = sombra(pols, n_ep=args.n_sombra, passos=args.passos,
                  semente=args.semente)
    ok_b = pareada(pols, n_ep=args.n, passos=args.passos,
                   semente=args.semente)

    print()
    if ok_a and ok_b:
        print('OK: as variantes em C sao indistinguiveis do torch em malha '
              'fechada.')
        print('A LUT esta liberada -- a decisao final e de latencia, e isso se')
        print('mede na placa com bench_politica.ino.')
        return 0
    if not ok_a:
        print('Bateria A reprovou: ha estado na trajetoria onde o C divergiu')
        print('mais de 1% de A_MAX. Investigue antes de olhar a bateria B.')
    if not ok_b:
        print('Bateria B reprovou: a variante se afastou MAIS que o braco de')
        print('tremor, ou seja, alem do que uma perturbacao de 1e-06 explica.')
        print('Se foi so a LUT, use tanhf na placa (--sem-lut para confirmar).')
    return 1


if __name__ == '__main__':
    sys.exit(main())
