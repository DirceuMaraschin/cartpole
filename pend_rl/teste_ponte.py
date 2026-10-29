#!/usr/bin/env python3
"""
Verifica, sem hardware, que a ponte PC<->ESP32 monta a MESMA observacao
que o simulador entrega ao agente.

Se este teste falhar, o agente vai ver um mundo diferente do que treinou,
e nenhuma quantidade de treinamento salva.
"""
import struct
import numpy as np

from pendulo_env import PenduloInvertido, PASSOS_POR_M, CURSO_M, PASSO_ENC
import pendulo_env as E
from rl_bridge import Ponte, crc8, FMT_SUBIDA, CONTAGENS_POR_VOLTA, ENC_SINAL


class PonteFake(Ponte):
    """Mesma logica da Ponte, sem abrir porta serial."""
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


def firmware_empacota(x_m, v, phi, omega, passos_por_m, limite_b, seq=0, flags=4):
    """Reproduz o que o ESP32 coloca no pacote a partir do estado fisico."""
    pos = int(round(x_m * passos_por_m + limite_b / 2))
    enc = int(round(phi / (2 * np.pi / CONTAGENS_POR_VOLTA) / ENC_SINAL))
    p = bytearray(b'\x55\xAA')
    p.append(seq & 0xFF)
    p += struct.pack('<i', pos)
    p += struct.pack('<i', enc)
    p += struct.pack('<f', v)
    p += struct.pack('<f', omega)
    p.append(flags)
    p.append(crc8(bytes(p)))
    return bytes(p)


def main():
    limite_b = int(round(CURSO_M * PASSOS_POR_M))
    ponte = PonteFake(PASSOS_POR_M, limite_b)

    env = PenduloInvertido(dominio_aleatorio=False, max_passos=1000,
                           semente=7, perturbacoes=False)
    obs_sim = env.reset()

    print('=== CONSISTENCIA SIMULADOR x PONTE ===')
    print(f'passos_por_m={PASSOS_POR_M:.0f}  limite_b={limite_b}  '
          f'passo do encoder={np.degrees(PASSO_ENC):.4f} deg\n')
    print(f'{"passo":>6} {"erro x":>10} {"erro v":>10} {"erro psi":>10} '
          f'{"erro psiDot":>12} {"erro a_ant":>11}')

    piores = np.zeros(5)
    a_ant = 0.0
    for k in range(300):
        # o firmware empacota o estado VERDADEIRO do simulador
        phi_q = np.round(env.phi / PASSO_ENC) * PASSO_ENC
        pkt = firmware_empacota(env.x, env.v, phi_q, env._om_f,
                                PASSOS_POR_M, limite_b, seq=k)
        _, seq, pos, enc, v, om, fl, _ = struct.unpack(FMT_SUBIDA, pkt)
        est = dict(seq=seq, pos=pos, enc=enc, v=v, omega=om, flags=fl)
        obs_ponte, x_p, psi_p = ponte.observar(est, a_ant)

        d = np.abs(obs_ponte - obs_sim)
        piores = np.maximum(piores, d)
        if k % 60 == 0:
            print(f'{k:6d} {d[0]:10.2e} {d[1]:10.2e} {d[2]:10.2e} '
                  f'{d[3]:12.2e} {d[4]:11.2e}')

        acao = np.array([np.sin(k * 0.1) * 0.3], dtype=np.float32)
        obs_sim, r, caiu, corte, info = env.step(acao)
        a_ant = info['a']
        if caiu or corte:
            obs_sim = env.reset()
            a_ant = 0.0

    # O erro residual esperado e METADE de um passo de quantizacao: a posicao
    # so existe em multiplos de 1/40000 m e o angulo em multiplos de 0,15 graus.
    # Comparar com zero absoluto nao faria sentido -- o hardware nao tem
    # resolucao infinita. O criterio e: o erro cabe dentro de meio passo?
    escalas = {
        'x':      (E.X_LIM,    1.0 / PASSOS_POR_M / 2, 'mm',  1000),
        'v':      (E.V_MAX,    1e-6,                   'mm/s', 1000),
        'psi':    (E.PSI_LIM,  PASSO_ENC / 2,          'graus', 180 / np.pi),
        'psiDot': (E.PSID_ESC, 1e-6,                   'rad/s', 1),
        'a_ant':  (E.A_MAX,    1 / 4000 / 2,           'm/s2',  1),
    }
    print('\npior erro por componente (em unidades fisicas):')
    todos_ok = True
    for (nome, e) in zip(escalas, piores):
        esc, tol, uni, k = escalas[nome]
        erro_fis = e * esc
        ok = erro_fis <= tol * 1.05
        todos_ok &= ok
        print(f'   {nome:7s} {erro_fis*k:9.5f} {uni:6s} '
              f'(meio passo = {tol*k:.5f})   {"OK" if ok else "FALHOU"}')

    if todos_ok:
        print('\nAPROVADO: toda divergencia cabe dentro de meio passo de quantizacao.')
        print('O agente vera no hardware exatamente o que viu no simulador.')
    else:
        print('\nREPROVADO: ha divergencia real. Confira as constantes em rl_bridge.py.')

    # --- verifica tambem o caminho da acao ---
    print('\n=== CAMINHO DA ACAO (PC -> firmware) ===')
    print(f'{"a enviada":>11} {"int16":>8} {"a reconstruida":>16} {"erro":>10}')
    for a in [-8.0, -3.333, -0.001, 0.0, 0.5, 2.718, 8.0]:
        bruto = int(np.clip(a, -8, 8) * 4000)
        bruto = max(-32768, min(32767, bruto))
        rec = bruto / 4000.0
        print(f'{a:11.4f} {bruto:8d} {rec:16.5f} {abs(rec-a):10.2e}')
    print('resolucao da acao: 1/4000 = 0,00025 m/s2 -- desprezivel frente a 8 m/s2')


if __name__ == '__main__':
    main()