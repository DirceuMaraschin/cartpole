#!/usr/bin/env python3
"""
ESP32 falso para testar a sequencia automatica SEM bancada.

Reproduz o que importa do firmware:
  - o watchdog de RL_TIMEOUT_MS = 100 ms (sai do modo RL se a acao nao chegar)
  - o comando 'o' como TOGGLE, podendo comecar LIGADO
  - as mensagens de texto com '#' e o modo binario depois do 'r'
  - a flag F_PERDEU quando a acao atrasa

Serve de rede de protecao para os dois bugs que derrubaram a etapa 3 em 29/10:
  1. a sequencia lia pacotes sem responder  -> o watchdog devolvia ao PID
  2. 'o' caia no estado errado sem ninguem notar -> o agente atuaria na etapa 3
"""
import os, pty, struct, sys, threading, time
import numpy as np

from rl_bridge import (crc8, CAB_SUBIDA, CAB_DESCIDA, TAM_SUBIDA,
                       F_RL, F_ZERO_OK, F_PERDEU, F_OBSERVA, CMD_PID,
                       ENC_SINAL, CONTAGENS_POR_VOLTA)

RL_TIMEOUT_MS = 100
DT = 0.020


class ESP32Falso(threading.Thread):
    """Fala o protocolo do firmware por um pty."""

    def __init__(self, fd, observa_inicial=False, swingup_s=0.6):
        super().__init__(daemon=True)
        self.fd = fd
        self.rlObserva = observa_inicial
        self.swingup_s = swingup_s
        self.modo = 'PID'
        self.zerado = False
        self.binario = False
        self.ultima_acao = time.time()
        self.seq = 0
        self.rodando = True
        self.buf = b''
        self.acoes = 0
        self.saidas_por_watchdog = 0
        self.observa_no_r = None
        self.capturou_em = None
        self.texto_pos_rl = 0
        self.supervisor = True
        self.supervisor_no_r = None
        self.ja_esteve_rl = False

    # ---------------- lado de baixo: o que o firmware imprime ----------------
    def diz(self, txt):
        os.write(self.fd, f'# {txt}\n'.encode())

    def pacote(self, flags):
        x, phi, v, om = 0.0279, np.pi - 0.0026, 0.02, 0.0
        pos = int(round(x * 40000 + 21485 / 2))
        # phi e medido do PENDURADO (=0); em pe vale pi. Estava errado aqui e
        # entregava psi = -179.9 graus ao agente.
        enc = int(round(phi / (2 * np.pi / CONTAGENS_POR_VOLTA) / ENC_SINAL))
        p = bytearray(CAB_SUBIDA)
        p.append(self.seq & 0xFF)
        p += struct.pack('<i', pos) + struct.pack('<i', enc)
        p += struct.pack('<f', v) + struct.pack('<f', om)
        p.append(flags)
        p.append(crc8(bytes(p)))
        self.seq += 1
        os.write(self.fd, bytes(p))

    # ---------------- lado de cima: o que o PC manda ----------------
    def processar_entrada(self):
        try:
            self.buf += os.read(self.fd, 4096)
        except (BlockingIOError, OSError):
            return
        if not self.binario:
            # FORA do modo binario o firmware le cada byte como uma TECLA.
            # E o que transforma um pacote de acao atrasado em comandos: na
            # bancada de 29/10, 21 bytes viraram um 'Z' e 18 "desconhecido".
            for c in self.buf:
                self.comando(chr(c))
            self.buf = b''
            return
        while self.buf:
            i = self.buf.find(CAB_DESCIDA)
            if i == 0 and len(self.buf) >= 7:
                _, _, seq, bruto, cmd, _ = (CAB_DESCIDA, None,
                                            self.buf[2], 0, 0, 0)
                seq = self.buf[2]
                bruto = struct.unpack('<h', self.buf[3:5])[0]
                cmd = self.buf[5]
                self.buf = self.buf[7:]
                self.ultima_acao = time.time()
                self.acoes += 1
                if cmd == CMD_PID and self.modo == 'RL':
                    self.modo = 'PID'
                    self.diz('Saindo do modo RL: pedido do PC')
                continue
            if i == 0:
                return                      # pacote binario incompleto
            corte = i if i > 0 else len(self.buf)
            for c in self.buf[:corte]:
                self.comando(chr(c))
            self.buf = self.buf[corte:]
            if i < 0:
                return

    def comando(self, c):
        if c in '\r\n ':
            return
        if self.ja_esteve_rl and self.modo != 'RL' and c != 's':
            # byte de acao lido como TECLA: foi isso que gerou o 'Z' e os 18
            # "comando desconhecido" na bancada em 29/10
            self.texto_pos_rl += 1
        if c == 'k':
            os.write(self.fd, b'passos_por_m=40000\ncurso_passos=21485\n')
        elif c == 'z':
            self.zerado = True
            self.diz('Zero definido')
        elif c == 'i':
            self.modo = 'SWING'
            self.capturou_em = time.time() + self.swingup_s
        elif c == 'm':
            self.supervisor = not self.supervisor
            self.diz('Supervisor PID: ' + ('LIGADO' if self.supervisor else 'DESLIGADO'))
        elif c == 'o':
            self.rlObserva = not self.rlObserva
            self.diz('Modo observacao: ' + ('LIGADO (o PID atua, o agente so assiste)'
                     if self.rlObserva else 'desligado (o AGENTE atua)'))
        elif c == 'r':
            if self.modo == 'EQUILIBRIO':
                self.modo = 'RL'
                self.binario = True
                self.ultima_acao = time.time()
                self.observa_no_r = self.rlObserva
                self.ja_esteve_rl = True
                self.supervisor_no_r = self.supervisor
                self.diz('Entregando ao agente')
            else:
                self.diz('Aguardando entrega: haste ainda nao estavel')
        elif c == 's':
            self.modo = 'PID'
            self.binario = False

    def run(self):
        prox = time.time()
        while self.rodando:
            self.processar_entrada()
            agora = time.time()
            if agora < prox:
                time.sleep(0.001)
                continue
            prox = agora + DT

            if self.modo == 'SWING' and agora >= self.capturou_em:
                self.modo = 'EQUILIBRIO'
                self.diz('Capturado!')

            atraso = (agora - self.ultima_acao) * 1000
            if self.modo == 'RL' and atraso > RL_TIMEOUT_MS:
                self.modo = 'PID'
                self.binario = False
                self.saidas_por_watchdog += 1
                self.diz('Saindo do modo RL: comunicacao perdida')

            flags = F_ZERO_OK if self.zerado else 0
            if self.modo == 'RL':
                flags |= F_RL
                if self.rlObserva:
                    flags |= F_OBSERVA
                if atraso > DT * 1000 * 1.5:
                    flags |= F_PERDEU
            if self.binario or self.modo in ('EQUILIBRIO', 'SWING'):
                self.pacote(flags)


def cenario(nome, observa_inicial, observar_pedido, atraso_abertura_ms=25.0,
            superv_inicial=True, superv_pedido=True):
    """atraso_abertura_ms: a lacuna REAL entre a sequencia terminar e a primeira
    acao do laco principal -- abrir o CSV, criar a thread do teclado e rodar a
    primeira inferencia. Na bancada de 29/10 essa primeira inferencia custou
    20,44 ms (rede fria). Sem modelar isso o teste nao enxerga o watchdog.
    """
    import rl_bridge
    mestre, escravo = pty.openpty()
    os.set_blocking(mestre, False)
    esp = ESP32Falso(mestre, observa_inicial=observa_inicial)
    esp.supervisor = superv_inicial
    esp.start()

    # Num pty a taxa e ficcao: nao existe fio, nenhum bit e temporizado por ela.
    # E no macOS pedir 921600 QUEBRA: por nao ser uma taxa padiao, o pyserial
    # recorre ao ioctl IOSSIOSPEED, que so existe para portas seriais de
    # verdade -- "OSError 25: Inappropriate ioctl for device". Usamos uma taxa
    # padrao aqui; na bancada a porta e real e os 921600 valem normalmente.
    p = None
    for baud in (115200, 9600):
        try:
            p = rl_bridge.Ponte(os.ttyname(escravo), baud)
            break
        except OSError:
            continue
    if p is None:
        raise RuntimeError('nao consegui abrir o pty em nenhuma taxa padrao')
    p.ler_constantes(espera=1.0)

    import builtins
    orig = builtins.input
    builtins.input = lambda *a, **k: ''
    try:
        passos_rl = 0
        ok = rl_bridge.sequencia_auto(p, observar=observar_pedido,
                                      supervisor=superv_pedido)
        if ok:
            time.sleep(atraso_abertura_ms / 1000.0)   # a lacuna do main()
            t0 = time.time()
            while time.time() - t0 < 1.0:
                est = p.ler_estado(0.05)
                if est:
                    p.enviar_acao(0.5, est['seq'])
                    if est['flags'] & F_RL:
                        passos_rl += 1
    finally:
        builtins.input = orig
        esp.rodando = False
        time.sleep(0.05)
        p.ser.close()
        os.close(mestre); os.close(escravo)

    return dict(nome=nome, ok=ok, obs_no_r=esp.observa_no_r,
                watchdog=esp.saidas_por_watchdog, acoes=esp.acoes,
                passos_rl=passos_rl, modo_final=esp.modo,
                superv_no_r=esp.supervisor_no_r)


def main():
    print('=== SEQUENCIA AUTOMATICA CONTRA UM ESP32 FALSO ===')
    print('(inclui o watchdog de 100 ms e o toggle "o")\n')

    casos = [
        # nome,                         obs ini, obs ped, sup ini, sup ped
        ('etapa 3, obs DESLIGADA',      False, True,  True,  True),
        ('etapa 3, obs JA LIGADA',      True,  True,  True,  True),  # seu caso
        ('etapa 4, obs DESLIGADA',      False, False, True,  True),
        ('etapa 4, obs JA LIGADA',      True,  False, True,  True),
        # Etapa 5: o supervisor SO pode ser desligado antes do 'r' -- em modo
        # RL o firmware nao le texto e o protocolo nao tem comando para ele.
        ('etapa 5, superv ja LIGADO',   True,  False, True,  False),
        ('etapa 5, superv ja DESLIG',   False, False, False, False),
        ('religar superv desligado',    False, True,  False, True),
    ]

    linhas = []
    for nome, oi, op, si, sp in casos:
        print(f'--- {nome} ---')
        r = cenario(nome, oi, op, superv_inicial=si, superv_pedido=sp)
        linhas.append((nome, op, sp, r))
        print()

    print('=== RESULTADO ===')
    print(f'{"cenario":30} {"obs":>6} {"ped":>6} {"sup":>6} {"ped":>6} '
          f'{"wdog":>5} {"passos/s":>9}')
    todos_ok = True
    for nome, op, sp, r in linhas:
        # passos/s: a 50 Hz o esperado e ~50. O log da bancada deu 2.
        bom = (r['ok'] and r['obs_no_r'] == op and r['superv_no_r'] == sp
               and r['watchdog'] == 0 and r['passos_rl'] >= 40)
        todos_ok &= bom
        print(f'{nome:30} {str(r["obs_no_r"]):>6} {str(op):>6} '
              f'{str(r["superv_no_r"]):>6} {str(sp):>6} '
              f'{r["watchdog"]:>5} {r["passos_rl"]:>9}   '
              f'{"OK" if bom else "FALHOU"}')

    print()
    if todos_ok:
        print('APROVADO: em todos os cenarios a observacao E o supervisor')
        print('ficaram no estado PEDIDO, o watchdog nunca disparou e a taxa')
        print('ficou perto de 50 Hz.')
    else:
        print('REPROVADO: ver a coluna que divergiu.')
        sys.exit(1)


if __name__ == '__main__':
    main()