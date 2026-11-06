#!/usr/bin/env python3
"""
Etapa 1 do passo 7: confere a rede EMBARCADA contra a do PC, amostra por amostra.

O QUE ISTO MEDE. Com o comando `l` o firmware calcula a ação na placa e manda
"NET <seq> <bruta>" como texto, junto do pacote binário de sempre. Este script
reconstrói a observação a partir do MESMO pacote, roda a MESMA rede em C (via
ctypes) e compara. Qualquer diferença acima do arredondamento de float32 é bug
de integração: escala trocada, entrada fora de ordem, sinal do encoder, x com
centro errado.

POR QUE COM `o` (OBSERVADOR). Quem atua é o PID. A rede só opina. Risco
mecânico zero, e o teste dura o que você quiser -- com o agente atuando, um
erro de sinal derruba a haste em 0,6 s e sobram 30 amostras.

A CADEIA DO a_ant. A placa alimenta a rede com `aRL`, que é a ação dela mesma
no ciclo anterior. Aqui a cadeia é montada a partir da `bruta` QUE A PLACA
REPORTOU, não da nossa. Se usássemos a nossa, um único passo divergente
contaminaria todos os seguintes e o relatório diria "tudo errado" em vez de
"um passo errado". Assim cada amostra é julgada isoladamente.

E ela é montada com UM PASSO DE ATRASO EXATO, o que exige avaliação preguiçosa.
A primeira versão deste arquivo errava aqui e reprovou a placa com 82% de erro:
a linha "NET k" só chega DEPOIS do pacote k, então calcular na hora em que o
pacote k é lido usa bruta[k-2], não bruta[k-1]. Uma entrada da observação
atrasada de um passo basta para mudar a ação de +0,29 para -0,53. Por isso o
laço guarda o pacote e só o avalia quando bruta[k] E bruta[k-1] chegaram.

Uso:
    python conferir_local.py --porta /dev/cu.usbserial-0001
    python conferir_local.py --porta ... --seg 60 --csv conf_01.csv
"""

import argparse
import os
import sys
import time

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)

from rl_bridge import Ponte, A_MAX, CMD_PARAR
from politica_c import PoliticaC

F_RL = 0x04
F_OBSERVA = 0x40
F_COMPLETO = 0x80


def esperar(p, trecho, seg, oque):
    """Espera uma mensagem do firmware. Devolve a mensagem ou None."""
    t0 = time.time()
    while time.time() - t0 < seg:
        p.ler_estado(prazo=0.05)
        for m in p.msgs:
            if trecho in m:
                return m
    print(f'  !! nao vi "{oque}" em {seg:.0f} s')
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--porta', required=True)
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--seg', type=float, default=30.0)
    ap.add_argument('--csv', default='')
    ap.add_argument('--modelo', default='modelos_sw/melhor.pt')
    ap.add_argument('--lut', action='store_true',
                    help='comparar contra a variante com LUT (use so se a placa '
                         'estiver gravada com POLITICA_TANH_LUT)')
    args = ap.parse_args()

    pol = PoliticaC(lut=args.lut)
    print(f'Politica local: {pol}')

    p = Ponte(args.porta, args.baud)
    p.ler_constantes()
    print(f'passos_por_m={p.passos_por_m:.1f} limite_b={p.limite_b}')
    if p.limite_b == 0:
        raise SystemExit('nao li o curso do firmware; reinicie a placa (EN)')

    # --- sequencia de entrada: l (rede local), o (observador), r (entrega) ---
    print('\n--- ligando rede local e modo observador ---')
    p.msgs.clear()
    p.texto('l')
    if not esperar(p, 'REDE LOCAL LIGADA', 3.0, 'rede local ligada'):
        raise SystemExit('a placa nao confirmou a rede local. Gravou o firmware novo?')
    p.msgs.clear()
    p.texto('o')
    esperar(p, 'observacao', 3.0, 'modo observacao')
    p.msgs.clear()
    print('--- entregando ao agente (o PID continua atuando) ---')
    p.texto('r')
    esperar(p, 'binario', 20.0, 'protocolo binario')

    # --- laco de comparacao ---
    #
    # JANELA, nao dicionario sem fim: seq tem 1 byte e da a volta a cada 256
    # amostras (5,12 s). Guardando mais que isso, um `seq` casaria com uma
    # bruta de 256 amostras atras e o relatorio ficaria mentindo. 64 e folga
    # larga -- o normal e resolver em uma ou duas iteracoes.
    JANELA = 64
    pacotes = {}    # seq -> est, esperando a bruta da placa e a do antecessor
    placa = {}      # seq -> bruta reportada pela placa
    pares = []
    t0 = time.time()
    n_pac = 0
    sem_flag = 0
    sem_antecessor = 0

    print(f'\n--- comparando por {args.seg:.0f} s ---')
    while time.time() - t0 < args.seg:
        est = p.ler_estado(prazo=0.05)
        if est is None:
            continue
        n_pac += 1
        if not (est['flags'] & F_RL):
            sem_flag += 1
            continue
        pacotes[est['seq']] = est

        # recolhe o que a placa reportou desde a ultima volta
        placa.update(p.net)
        p.net.clear()

        # avalia os pacotes cujo lado da placa E cujo antecessor ja chegaram
        for seq in sorted(pacotes):
            ant = (seq - 1) & 0xFF
            if seq not in placa or ant not in placa:
                continue
            e = pacotes.pop(seq)
            # a_ant = a acao que estava valendo na placa naquele ciclo, que e a
            # bruta do ciclo ANTERIOR, saturada e escalada -- exatamente o que
            # politicaDaPlaca recebe em `aRL`.
            a_ant = float(np.clip(placa[ant], -1.0, 1.0)) * A_MAX
            obs, x, psi = p.observar_sw(e, a_ant)
            pares.append((seq, float(pol.agir(obs)[0]), placa[seq],
                          obs.copy(), x, psi, e['flags']))

        # descarta o que envelheceu: sem o antecessor nao da para comparar, e
        # isso e sintoma de pacote perdido, nao de divergencia.
        while len(pacotes) > JANELA:
            pacotes.pop(next(iter(pacotes)))
            sem_antecessor += 1
        while len(placa) > JANELA:
            placa.pop(next(iter(placa)))

    p.enviar_acao(0.0, 0, CMD_PARAR)
    time.sleep(0.2)

    # --- relatorio ---
    print(f'\n=== RESULTADO ===')
    print(f'pacotes             {n_pac} em {time.time()-t0:.1f} s '
          f'({n_pac/(time.time()-t0):.1f} Hz)')
    print(f'fora do modo RL     {sem_flag}')
    print(f'descartados sem antecessor  {sem_antecessor}  (sintoma de perda)')
    print(f'ainda na janela     {len(pacotes)} pacotes, {len(placa)} brutas')
    print(f'pares comparados    {len(pares)}')
    print(f'CRC ruim            {p.crc_ruim} | pacotes perdidos {p.perdidos}')

    if not pares:
        print('\nNENHUM PAR. Verifique:')
        print('  - a placa esta com o firmware novo (comando l existe)?')
        print('  - as linhas NET estao saindo? rode com a ponte em modo verboso')
        return 1

    seqs = np.array([q[0] for q in pares])
    minhas = np.array([q[1] for q in pares])
    placas = np.array([q[2] for q in pares])
    flags = np.array([q[6] for q in pares])

    e = np.abs(minhas - placas)
    a_minha = np.clip(minhas, -1, 1) * A_MAX
    a_placa = np.clip(placas, -1, 1) * A_MAX
    ea = np.abs(a_minha - a_placa)
    i = int(np.argmax(e))

    print(f'\nerro em bruta       max {e.max():.3e}  medio {e.mean():.3e}')
    print(f'erro em m/s2        max {ea.max():.3e} de {A_MAX}  '
          f'({100*ea.max()/A_MAX:.4f}%)')
    print(f'faixa de bruta      [{placas.min():+.4f}, {placas.max():+.4f}]')
    print(f'observador ativo    {100*np.mean((flags & F_OBSERVA) != 0):.0f}% '
          f'das amostras')

    print(f'\npior par: seq {seqs[i]}')
    print(f'  obs   = [{", ".join(f"{v:+.6f}" for v in pares[i][3])}]')
    print(f'  x     = {1000*pares[i][4]:+.3f} mm   '
          f'psi = {np.degrees(pares[i][5]):+.3f} graus')
    print(f'  minha = {minhas[i]:+.7f}')
    print(f'  placa = {placas[i]:+.7f}')

    if args.csv:
        import csv
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['seq', 'x_mm', 'psi_graus', 'bruta_pc', 'bruta_placa',
                        'erro', 'flags'] + [f'obs{k}' for k in range(6)])
            for q in pares:
                w.writerow([q[0], 1000*q[4], np.degrees(q[5]),
                            f'{q[1]:.7f}', f'{q[2]:.7f}',
                            f'{abs(q[1]-q[2]):.3e}', q[6]]
                           + [f'{v:.7f}' for v in q[3]])
        print(f'\ngravado em {args.csv}')

    # --- veredicto ---
    print()
    if e.max() < 1e-5:
        print('APROVADO: a placa reproduz o PC dentro do arredondamento de')
        print('float32. A integracao esta correta -- pode seguir para a etapa 2')
        print('(equilibrio com supervisor ligado).')
        return 0
    if e.max() < 1e-2:
        print(f'ATENCAO: erro de {e.max():.2e}, acima do arredondamento mas')
        print('pequeno. Candidatos, em ordem de probabilidade:')
        print('  - centro do trilho: limiteB/2 inteiro vs 2.0f (meio passo)')
        print('  - uma constante de escala diferente entre politica.h e o PC')
        print('  - a cadeia de a_ant dessincronizada por um passo')
        return 1
    print(f'REPROVADO: erro de {e.max():.2e}. Isso nao e precisao.')
    print('Compare o vetor obs do pior par com o que a placa deveria ver:')
    print('  - duas entradas trocadas de lugar (ordem do contrato)')
    print('  - ENC_SINAL ou o sentido de psi invertido')
    print('  - X_LIM, V_MAX, PSID_ESC ou A_MAX diferentes dos dois lados')
    return 2


if __name__ == '__main__':
    sys.exit(main())
