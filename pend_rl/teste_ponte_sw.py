#!/usr/bin/env python3
"""
Verifica, sem hardware, que a ponte monta a MESMA observacao de 6 entradas
que o simulador do agente COMPLETO entrega.

A ordem e as escalas das 6 entradas sao o contrato com a rede treinada.
Trocar duas de lugar nao da erro de execucao: da um agente que decide ao
contrario, e isso so apareceria com a haste girando na bancada.
"""
import struct
import sys

import numpy as np

sys.path.insert(0, '.')

import pendulo_env as E
from pendulo_env_sw import PenduloSwingUp
from rl_bridge import Ponte, FMT_SUBIDA
from teste_ponte import firmware_empacota


class PonteFakeSW(Ponte):
    def __init__(self, passos_por_m, limite_b):
        self.passos_por_m = passos_por_m
        self.limite_b = limite_b
        self.seq_ant = None
        self.perdidos = 0
        self.buf = b''
        self.msgs = []
        self.bruto = []
        self.crc_ruim = 0
        self.verboso = False


def main():
    lb = int(round(E.CURSO_M * E.PASSOS_POR_M))
    pon = PonteFakeSW(E.PASSOS_POR_M, lb)
    env = PenduloSwingUp(dominio_aleatorio=False, perturbacoes=False,
                         semente=3, max_passos=900, frac_topo=0.0)
    obs_sim = env.reset(pendurado=True)

    print('=== CONSISTENCIA SIMULADOR x PONTE (agente completo, 6 entradas) ===')
    print('Percorre a ROTACAO INTEIRA: e onde sin/cos tem de bater, nao so')
    print('perto do equilibrio.\n')

    pior = np.zeros(6)
    a_ant = 0.0
    for k in range(900):
        phi_q = np.round(env.phi / E.PASSO_ENC) * E.PASSO_ENC
        pkt = firmware_empacota(env.x, env.v, phi_q, env._om_f,
                                E.PASSOS_POR_M, lb, seq=k % 256)
        _, sq, pos, enc, v, om, fl, _ = struct.unpack(FMT_SUBIDA, pkt)
        est = dict(seq=sq, pos=pos, enc=enc, v=v, omega=om, flags=fl)
        obs, x, psi = pon.observar_sw(est, a_ant)
        pior = np.maximum(pior, np.abs(obs - obs_sim))
        obs_sim, r, caiu, corte, info = env.step([np.sin(k*0.05)*0.5])
        a_ant = info['a']
        if caiu or corte:
            break

    # A tolerancia e meio passo de quantizacao, propagado para cada entrada.
    # Em sin/cos, meio passo de encoder vale sin(PASSO_ENC/2).
    meio_enc = np.sin(E.PASSO_ENC / 2)
    nomes = ['x/X_LIM', 'v/V_MAX', 'sin(psi)', 'cos(psi)', 'psiDot', 'a_ant']
    tol = [1/E.PASSOS_POR_M/2/E.X_LIM, 1e-6, meio_enc, meio_enc,
           1e-6, 1/4000/2/E.A_MAX]

    print(f'{"entrada":>10} {"pior erro":>12} {"tolerancia":>12}')
    ok = True
    for n_, e, t_ in zip(nomes, pior, tol):
        bom = e <= t_ * 1.6
        ok &= bom
        print(f'{n_:>10} {e:12.2e} {t_:12.2e}   {"OK" if bom else "FALHOU"}')

    print()
    if ok:
        print('APROVADO: a ponte monta a mesma observacao que o simulador,')
        print('em toda a rotacao. O agente vera no hardware o que viu treinando.')
    else:
        print('REPROVADO: confira a ordem e as escalas em Ponte.observar_sw')
        print('contra PenduloSwingUp._observar.')
        sys.exit(1)


if __name__ == '__main__':
    main()
