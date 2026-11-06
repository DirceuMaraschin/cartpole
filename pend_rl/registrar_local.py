#!/usr/bin/env python3
"""
Etapas 2 e 3 do passo 7: o agente EMBARCADO atuando, com o PC só registrando.

    --etapa 2   equilibrio, haste levantada a mao, SUPERVISOR LIGADO
    --etapa 3   agente completo, haste pendurada, supervisor desligado por 'g'

O PC NAO ENTRA NO LACO. Este script não manda ação nenhuma: só configura, lê e
grava. O watchdog do host está desativado no firmware quando a rede é local,
então fechar este script não para o pêndulo -- use `s` no fim (ele manda).

POR QUE OS ALTERNADORES SAO FORCADOS. `l`, `o` e `m` são toggles no firmware:
depois de uma corrida de conferência, `rlObserva` fica LIGADO e a etapa 2
rodaria com o PID atuando, parecendo um sucesso do agente que não aconteceu.
Cada tecla imprime o estado resultante, então aqui elas são enviadas e
CONFERIDAS, repetindo se preciso. Não depende de reset nem de memória do
operador.

A CONFERENCIA CONTINUA RODANDO. As linhas NET vêm de graça, então a comparação
bit a bit da etapa 1 segue ativa durante a operação -- agora em ângulos e
velocidades que o teste com o PID segurando nunca visitou. Se a integração
tivesse um erro que só aparece longe do topo, é aqui que ele apareceria.

Uso:
    python registrar_local.py --porta /dev/cu.usbserial-0001 --etapa 2 --seg 60
    python registrar_local.py --porta ... --etapa 3 --seg 120 --csv et3_01.csv
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)

from rl_bridge import Ponte, A_MAX, CMD_PARAR
from politica_c import PoliticaC

F_RL = 0x04
F_SUPERV = 0x08
F_PERDEU = 0x10
F_OBSERVA = 0x40
F_COMPLETO = 0x80

PE = np.radians(10.0)          # "em pe", igual as etapas anteriores
JANELA = 64


# ======================================================================
# controle dos alternadores
# ======================================================================
def _ler_por(p, seg):
    """Le a serial por um tempo, so para encher p.msgs."""
    t0 = time.time()
    while time.time() - t0 < seg:
        p.ler_estado(prazo=0.02)


def garantir(p, tecla, marca_ligado, quero, nome, tentativas=3):
    """Leva um alternador do firmware ao estado desejado, conferindo.

    marca_ligado: trecho que aparece na mensagem QUANDO LIGADO.
    """
    for k in range(tentativas):
        p.msgs.clear()
        p.texto(tecla)
        _ler_por(p, 0.6)
        vistas = [m for m in p.msgs if nome.lower() in m.lower()]
        if not vistas:
            continue
        esta = marca_ligado.lower() in vistas[-1].lower()
        if esta == quero:
            print(f'  {nome}: {"LIGADO" if esta else "desligado"} (ok)')
            return True
        print(f'  {nome}: veio {"LIGADO" if esta else "desligado"}, '
              f'queria {"LIGADO" if quero else "desligado"} -- alternando')
    print(f'  !! nao consegui por {nome} em '
          f'{"LIGADO" if quero else "desligado"}')
    return False


def esperar(p, trecho, seg, oque):
    t0 = time.time()
    while time.time() - t0 < seg:
        p.ler_estado(prazo=0.05)
        for m in p.msgs:
            if trecho in m:
                return m
    print(f'  !! nao vi "{oque}" em {seg:.0f} s')
    return None


# ======================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--porta', required=True)
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--etapa', type=int, choices=(2, 3), required=True)
    ap.add_argument('--seg', type=float, default=60.0)
    ap.add_argument('--csv', default='')
    ap.add_argument('--lut', action='store_true')
    ap.add_argument('--deixar-rodando', action='store_true',
                    help='sai SEM mandar parar: prova que o PC nao esta no '
                         'laco. Para parar depois, aperte EN na placa')
    args = ap.parse_args()

    pol = PoliticaC(lut=args.lut)
    p = Ponte(args.porta, args.baud)
    p.ler_constantes()
    print(f'passos_por_m={p.passos_por_m:.1f} limite_b={p.limite_b}')
    if p.limite_b == 0:
        raise SystemExit('nao li o curso do firmware; aperte EN e tente de novo')

    # --- alternadores num estado conhecido ---
    print(f'\n--- configurando para a etapa {args.etapa} ---')
    if not garantir(p, 'l', 'REDE LOCAL LIGADA', True, 'rede local'):
        raise SystemExit('sem rede local nao da para rodar; gravou o firmware novo?')
    garantir(p, 'o', 'LIGADO', False, 'Modo observacao')
    if args.etapa == 2:
        garantir(p, 'm', 'LIGADO', True, 'Supervisor PID')

    binario = False
    if args.etapa == 2:
        print('\n--- etapa 2: equilibrio, supervisor LIGADO ---')
        print('    LEVANTE A HASTE A MAO quando o firmware pedir.')
        print('    Mao na chave de emergencia.')
        p.msgs.clear()
        p.texto('r')
        if esperar(p, 'binario', 60.0, 'protocolo binario'):
            binario = True
    else:
        print('\n--- etapa 3: agente completo, haste PENDURADA e LIVRE ---')
        print('    O supervisor de angulo nao vale aqui: |psi| passa por 180.')
        print('    Protecao: trilho, velocidade, aceleracao, fins de curso.')
        print('    MAO NA CHAVE DE EMERGENCIA.')
        time.sleep(2.0)
        p.msgs.clear()
        p.texto('g')
        if esperar(p, 'AGENTE COMPLETO', 5.0, 'agente completo'):
            binario = True

    if not binario:
        print('\nnao entrou no modo binario. Abortando sem gravar.')
        p.texto('s')
        return 1

    # --- laco de registro ---
    pacotes, placa, linhas = {}, {}, []
    t0 = time.time()
    n_pac = 0
    saiu = None
    intervencoes = 0
    erro_max = 0.0

    print(f'\n--- registrando por {args.seg:.0f} s (Ctrl+C para parar) ---')
    try:
        while time.time() - t0 < args.seg:
            est = p.ler_estado(prazo=0.05)

            # o supervisor assumindo tira o firmware do binario: os pacotes
            # param e o texto volta. Nao e timeout -- e a rede de protecao.
            for m in p.msgs:
                if 'SUPERVISOR' in m or 'Parado' in m:
                    intervencoes += 1
                    saiu = m
            p.msgs.clear()
            if saiu:
                break
            if est is None:
                continue

            n_pac += 1
            if not (est['flags'] & F_RL):
                continue
            pacotes[est['seq']] = (time.time() - t0, est)
            placa.update(p.net)
            p.net.clear()

            for seq in sorted(pacotes):
                ant = (seq - 1) & 0xFF
                if seq not in placa or ant not in placa:
                    continue
                t, e = pacotes.pop(seq)
                a_ant = float(np.clip(placa[ant], -1.0, 1.0)) * A_MAX
                obs, x, psi = p.observar_sw(e, a_ant)
                minha = float(pol.agir(obs)[0])
                erro_max = max(erro_max, abs(minha - placa[seq]))
                linhas.append((t, seq, x, psi, e['v'], e['omega'],
                               placa[seq], minha, e['flags']))

            while len(pacotes) > JANELA:
                pacotes.pop(next(iter(pacotes)))
            while len(placa) > JANELA:
                placa.pop(next(iter(placa)))
    except KeyboardInterrupt:
        print('\n  interrompido pelo operador')

    # --- parar com seguranca ---
    # Em modo binario o firmware so le o pacote de 7 bytes; em texto, ele leria
    # esses 7 bytes como teclas digitadas. Por isso a escolha importa.
    if args.deixar_rodando and not saiu:
        print('\n  NAO enviei comando de parada: o pendulo segue por conta da')
        print('  placa. Se ele continuar equilibrando com este processo')
        print('  encerrado, esta provado que o PC nao esta no laco.')
        print('  PARA PARAR: aperte EN na placa, ou a chave de emergencia.')
        print('  (em modo binario o firmware NAO le teclas de texto, entao')
        print('   mandar "s" por um monitor nao funciona)')
    elif saiu:
        p.texto('s')
    else:
        p.enviar_acao(0.0, 0, CMD_PARAR)
    time.sleep(0.3)
    _ler_por(p, 0.5)

    # ---------------- relatorio ----------------
    dur = time.time() - t0
    print(f'\n=== ETAPA {args.etapa} ===')
    print(f'duracao             {dur:.1f} s')
    print(f'pacotes             {n_pac} ({n_pac/dur:.1f} Hz)')
    print(f'amostras no RL      {len(linhas)}')
    print(f'CRC ruim            {p.crc_ruim} | perdidos {p.perdidos}')
    print(f'conferencia bit a bit: erro max {erro_max:.3e} '
          f'{"(ok)" if erro_max < 1e-4 else "<< INVESTIGAR"}')
    if saiu:
        print(f'SAIU DO MODO RL     {saiu}')
    print(f'intervencoes        {intervencoes}')

    if not linhas:
        print('\nnenhuma amostra no modo RL.')
        return 1

    t = np.array([l[0] for l in linhas])
    x = np.array([l[2] for l in linhas])
    psi = np.array([l[3] for l in linhas])
    br = np.array([l[6] for l in linhas])
    fl = np.array([l[8] for l in linhas])

    em_pe = np.abs(psi) < PE
    print(f'\ntempo no RL         {t[-1]-t[0]:.1f} s')
    print(f'em pe (|psi|<10)    {100*em_pe.mean():.1f}% das amostras')
    if em_pe.any():
        print(f'<|psi|> em pe       {np.degrees(np.abs(psi[em_pe])).mean():.2f} graus'
              f'   (agente v1 na bancada: 1,46)')
    print(f'<|x|>               {1000*np.abs(x).mean():.1f} mm'
          f'   (agente v1 na bancada: 17,6)')
    print(f'|x| maximo          {1000*np.abs(x).max():.1f} mm   (limite 240)')
    print(f'|bruta| maximo      {np.abs(br).max():.3f}   '
          f'saturou em {100*np.mean(np.abs(br)>=1.0):.1f}% das amostras')
    print(f'F_PERDEU            {int((fl & F_PERDEU != 0).sum())} amostras')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'seq', 'x_mm', 'psi_graus', 'v_ms', 'omega_rads',
                        'bruta_placa', 'bruta_pc', 'erro', 'flags'])
            for l in linhas:
                w.writerow([f'{l[0]:.3f}', l[1], f'{1000*l[2]:.3f}',
                            f'{np.degrees(l[3]):.3f}', f'{l[4]:.4f}',
                            f'{l[5]:.4f}', f'{l[6]:.7f}', f'{l[7]:.7f}',
                            f'{abs(l[6]-l[7]):.3e}', l[8]])
        print(f'\ngravado em {args.csv}')

    # --- veredicto ---
    print()
    if erro_max > 1e-4:
        print('A conferencia bit a bit saiu da faixa durante a operacao.')
        print('Ha estado em que placa e PC discordam -- investigue antes de')
        print('seguir, mesmo que o pendulo tenha ficado em pe.')
        return 1
    if args.etapa == 2:
        ok = (not saiu) and em_pe.mean() > 0.95 and 1000*np.abs(x).max() < 230
        print('APROVADO: equilibrio sustentado sem intervencao do supervisor.'
              if ok else
              'NAO FECHOU: veja se saiu do RL, se ficou menos de 95% em pe ou '
              'se |x| encostou em 230 mm.')
        if ok:
            print('Proximo: --etapa 3 (agente completo, haste pendurada).')
    else:
        print(f'Etapa 3: subiu e ficou em pe {100*em_pe.mean():.0f}% do tempo.')
        print('Compare com a bancada via ponte: 90% em pe, <|psi|> 0,89 graus,')
        print('subida em 2,47 s, uma queda a cada 45 s.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
