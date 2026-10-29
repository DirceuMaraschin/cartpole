#!/usr/bin/env python3
"""
Ponte entre o agente PPO (no PC) e o pendulo fisico (ESP32).

Fecha a malha a 50 Hz: le o estado pela serial, roda a rede, devolve a acao.

Uso:
    python rl_bridge.py --porta /dev/cu.usbserial-0001 --modelo modelos/melhor.pt
    python rl_bridge.py --porta COM5 --malha-aberta     # NAO move o motor
    python rl_bridge.py --porta COM5 --segundos 30 --log logs/rl_01.csv

Teclas durante a execucao (digite + ENTER):
    r  entrega o controle ao agente       p  devolve ao PID
    s  parar                              q  encerrar
    v  limitador de velocidade            m  liga/desliga supervisor
"""

import argparse, csv, os, queue, struct, sys, threading, time
import numpy as np

try:
    import serial
except ImportError:
    sys.exit('Falta pyserial:  pip install pyserial')

from ppo import AtorCritico
import torch

# --- constantes que DEVEM bater com pendulo_env.py ---
X_LIM    = 0.24
V_MAX    = 0.45
PSI_LIM  = np.radians(35.0)
PSID_ESC = 6.0
A_MAX    = 8.0
CONTAGENS_POR_VOLTA = 2400
ENC_SINAL = -1

CAB_SUBIDA  = b'\x55\xAA'
CAB_DESCIDA = b'\x5A\xA5'
FMT_SUBIDA  = '<HBiiffBB'      # 21 bytes
TAM_SUBIDA  = 21

CMD_NADA, CMD_PARAR, CMD_PID, CMD_RL = 0, 1, 2, 3
F_FC_A, F_FC_B, F_RL, F_SUPERV, F_PERDEU, F_ZERO_OK, F_OBSERVA = 1, 2, 4, 8, 16, 32, 64


def crc8(dados):
    c = 0
    for b in dados:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


class Ponte:
    def __init__(self, porta, baud=921600):
        self.ser = serial.Serial(porta, baud, timeout=0.05)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.passos_por_m = 40000.0
        self.limite_b = 0
        self.seq_ant = None
        self.perdidos = 0
        self.buf = b''
        self.msgs = []
        self.verboso = False

    # ------------- cabecalho em texto, antes do modo binario -------------
    def ler_constantes(self, espera=3.0):
        """Le as constantes que o firmware imprime, para converter passos em metros."""
        self.ser.write(b'k')
        t0 = time.time()
        achou = {}
        while time.time() - t0 < espera:
            linha = self.ser.readline().decode('utf-8', 'replace').strip()
            if not linha:
                continue
            print(linha)
            if 'passos_por_m=' in linha:
                self.passos_por_m = float(linha.split('=')[1]); achou['p'] = 1
            elif 'curso_passos=' in linha:
                self.limite_b = int(linha.split('=')[1]); achou['c'] = 1
            if len(achou) == 2:
                break
        if len(achou) < 2:
            print('AVISO: nao li as constantes do firmware; usando padrao 40000 passos/m')
        print(f'>> passos_por_m={self.passos_por_m:.0f}  limite_b={self.limite_b}')

    # ------------------------- protocolo binario -------------------------
    def _mostrar_texto(self):
        """Imprime as linhas de texto do firmware que estao no buffer.

        O firmware so fala binario depois do comando r. Antes disso ele manda
        telemetria e mensagens ("# Capturado!", "# Modo observacao: LIGADO")
        que o operador PRECISA ver para saber quando apertar a proxima tecla.
        Sem isso voce digita as sequencia no escuro.
        """
        while b'\n' in self.buf:
            # nao engole um pacote binario que comece antes da quebra de linha
            iq = self.buf.find(CAB_SUBIDA)
            iln = self.buf.find(b'\n')
            if 0 <= iq < iln:
                return
            linha, self.buf = self.buf[:iln], self.buf[iln + 1:]
            txt = linha.decode('utf-8', 'replace').strip()
            if not txt:
                continue
            if txt.startswith('#'):
                self.msgs.append(txt)
                print(f'  [esp32] {txt}')
            elif self.verboso:
                # Telemetria do firmware (PID/SWING/INDO/ESPERA). Por padrao
                # fica escondida: a 5 Hz ela inunda o terminal e voce nao
                # enxerga mais o que digita nem as mensagens que importam.
                print(f'  [esp32] {txt}')

    def ler_estado(self, prazo=0.05):
        """Le um pacote de estado. Devolve None se nao vier nada valido.

        O buffer e persistente entre chamadas: um pacote pode chegar partido
        em duas leituras, e descartar o buffer perderia metade dele.
        """
        t0 = time.time()
        while time.time() - t0 < prazo:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
            i = self.buf.find(CAB_SUBIDA)
            if i >= 0 and len(self.buf) - i >= TAM_SUBIDA:
                if i > 0:
                    self._mostrar_texto()
                    i = self.buf.find(CAB_SUBIDA)
                    if i < 0 or len(self.buf) - i < TAM_SUBIDA:
                        continue
                p = self.buf[i:i + TAM_SUBIDA]
                if crc8(p[:20]) != p[20]:
                    self.buf = self.buf[i + 2:]
                    continue
                self.buf = self.buf[i + TAM_SUBIDA:]
                _, seq, pos, enc, v, om, flags, _ = struct.unpack(FMT_SUBIDA, p)
                if self.seq_ant is not None:
                    falta = (seq - self.seq_ant - 1) & 0xFF
                    self.perdidos += falta
                self.seq_ant = seq
                return dict(seq=seq, pos=pos, enc=enc, v=v, omega=om, flags=flags)
            self._mostrar_texto()
            if len(self.buf) > 4096:        # nao deixa o buffer crescer sem fim
                self.buf = self.buf[-512:]
            time.sleep(0.0005)
        return None

    def enviar_acao(self, a_ms2, seq=0, cmd=CMD_NADA):
        bruto = int(np.clip(a_ms2, -A_MAX, A_MAX) * 4000)
        bruto = max(-32768, min(32767, bruto))
        p = bytearray(CAB_DESCIDA)
        p.append(seq & 0xFF)
        p += struct.pack('<h', bruto)
        p.append(cmd)
        p.append(crc8(p[:6]))
        self.ser.write(p)

    def texto(self, c):
        self.ser.write(c.encode()[:1])

    # --------------------------- observacao ---------------------------
    def observar(self, est, a_ant):
        x = (est['pos'] - self.limite_b / 2) / self.passos_por_m
        phi = ENC_SINAL * est['enc'] * (2 * np.pi / CONTAGENS_POR_VOLTA)
        psi = wrap(np.pi - phi)
        obs = np.array([x / X_LIM,
                        est['v'] / V_MAX,
                        psi / PSI_LIM,
                        -est['omega'] / PSID_ESC,
                        a_ant / A_MAX], dtype=np.float32)
        return obs, x, psi


def sequencia_auto(p, observar=True):
    """Faz z -> i -> espera captura -> centraliza -> o -> r sozinho.

    Digitar comandos enquanto o firmware fala e a receita para errar o tempo.
    Aqui cada passo espera a RESPOSTA do ESP32 antes do proximo.
    """
    def bombear(seg):
        t0 = time.time()
        while time.time() - t0 < seg:
            p.ler_estado(0.02)

    def esperar(trecho, seg, oque):
        t0 = time.time()
        while time.time() - t0 < seg:
            p.ler_estado(0.02)
            if any(trecho in m for m in p.msgs):
                p.msgs.clear()
                return True
        print(f'  >> TEMPO ESGOTADO esperando: {oque}')
        return False

    print()
    print('=== SEQUENCIA AUTOMATICA ===')
    p.texto('s'); bombear(0.5); p.msgs.clear()

    input('  Deixe a haste PENDURADA e PARADA, depois tecle Enter: ')
    p.msgs.clear()
    p.texto('z')
    if not esperar('Zero definido', 3, 'o zero do encoder'):
        return False
    print('  zero definido')

    p.texto('i')
    print('  swing-up em andamento...')
    if not esperar('Capturado', 30, 'a captura do pendulo'):
        print('  >> O swing-up nao capturou. Confira ALVO_EN e o atrito do pivo.')
        return False
    print('  capturado; aguardando o carrinho centralizar')
    bombear(4.0)

    if observar:
        p.msgs.clear()
        p.texto('o')
        if not esperar('observacao', 3, 'a confirmacao do modo observacao'):
            return False
        if any('desligado' in m for m in p.msgs):
            p.msgs.clear(); p.texto('o'); esperar('observacao', 3, 'observacao')
        print('  modo observacao LIGADO (o PID atua, o agente so assiste)')

    p.msgs.clear()
    p.texto('r')
    t0 = time.time()
    while time.time() - t0 < 15:
        est = p.ler_estado(0.05)
        if est and (est['flags'] & F_RL):
            print('  modo RL ativo -- coletando')
            print()
            return True
    print('  >> O firmware nao entrou no modo RL.')
    print('     Veja as mensagens "Aguardando entrega" acima: elas dizem o que falta.')
    return False


def ler_teclado(fila):
    """Le comandos do teclado.

    Usa readline() e NAO 'for linha in sys.stdin': o iterador de arquivo faz
    read-ahead em bloco, e num terminal interativo isso faz a linha digitada
    ficar presa no buffer ate o bloco encher. O sintoma e exatamente "digitei
    e nao aconteceu nada".
    """
    while True:
        linha = sys.stdin.readline()
        if not linha:            # EOF: o terminal fechou
            fila.put('q')
            return
        fila.put(linha.strip())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--porta', required=True)
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--modelo', default='modelos/melhor.pt')
    ap.add_argument('--segundos', type=float, default=0, help='0 = sem limite')
    ap.add_argument('--log', default='')
    ap.add_argument('--malha-aberta', action='store_true',
                    help='calcula a acao mas envia ZERO: testa o laco sem mover o motor')
    ap.add_argument('--auto', action='store_true',
                    help='executa z/i/o/r sozinho, esperando a resposta de cada passo')
    ap.add_argument('--verboso', action='store_true',
                    help='mostra tambem a telemetria do firmware (inunda o terminal)')
    ap.add_argument('--psi-abortar', type=float, default=18.0,
                    help='graus: acima disso o PC devolve o controle ao PID')
    args = ap.parse_args()

    rede = AtorCritico(dim_obs=5, dim_acao=1)
    rede.load_state_dict(torch.load(args.modelo, map_location='cpu',
                                    weights_only=True)['rede'])
    rede.eval()
    print(f'Modelo carregado: {args.modelo}')
    if args.malha_aberta:
        print('*** MALHA ABERTA: o motor NAO sera comandado pelo agente ***')

    p = Ponte(args.porta, args.baud)
    p.ler_constantes()

    p.verboso = args.verboso

    if args.auto:
        if not sequencia_auto(p, observar=not args.malha_aberta):
            print('\nA sequencia nao completou. Nada foi coletado.')
            try:
                p.texto('s'); time.sleep(0.1); p.ser.close()
            except Exception:
                pass
            return

    fila = queue.Queue()
    threading.Thread(target=ler_teclado, args=(fila,), daemon=True).start()
    if not args.auto:
        print('\nComandos no proprio terminal (tecla + ENTER): z i o r s q')
        print('A telemetria do firmware esta escondida; use --verboso para ve-la.\n')

    esc = None
    if args.log:
        os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
        f = open(args.log, 'w', newline='')
        esc = csv.writer(f)
        esc.writerow(['t_s', 'x_m', 'v_ms', 'psi_rad', 'psiDot_rads',
                      'a_ms2', 'flags', 'seq'])

    a_ant = 0.0
    n = 0
    t0 = time.time()
    t_ult = t0
    lat = []
    psis = []
    comp = []
    em_rl = False

    try:
        while True:
            while not fila.empty():
                c = fila.get().lower()
                if c == 'q':
                    raise KeyboardInterrupt
                if c:
                    p.texto(c[0])
                    print(f'  [enviado: {c[0]}]')

            est = p.ler_estado()
            if est is None:
                if em_rl:
                    print('  !! sem pacotes do ESP32')
                time.sleep(0.01)
                continue

            agora = time.time()
            em_rl = bool(est['flags'] & F_RL)
            if not em_rl:
                # firmware ainda em PID/homing: so acompanha
                time.sleep(0.002)
                continue

            obs, x, psi = p.observar(est, a_ant)

            # --- rede neural ---
            ti = time.time()
            acao, _, _ = rede.agir(obs, deterministico=True)
            lat.append((time.time() - ti) * 1000)
            a = float(np.clip(acao[0], -1, 1)) * A_MAX

            # No modo observacao o PID esta atuando. Reconstruimos a acao dele
            # a partir do mesmo estado (sem o termo integral, que e pequeno) e
            # comparamos: se os sinais discordarem sistematicamente, alguma
            # convencao esta invertida -- e melhor descobrir aqui do que com o
            # agente no comando.
            observando = bool(est['flags'] & F_OBSERVA)
            if observando:
                a_pid = float(np.clip(40.0*psi + 5.5*(-est['omega'])
                                      + 5.0*x + 17.5*est['v'], -A_MAX, A_MAX))
                comp.append((a, a_pid))

            # --- protecao do lado do PC (a 3a camada; o firmware tem as outras) ---
            cmd = CMD_NADA
            if abs(np.degrees(psi)) > args.psi_abortar:
                cmd = CMD_PID
                print(f'  >> PC abortando: psi = {np.degrees(psi):.1f} graus')

            p.enviar_acao(0.0 if args.malha_aberta else a, est['seq'], cmd)
            a_ant = a
            n += 1
            psis.append(abs(np.degrees(psi)))

            if esc:
                esc.writerow([round(agora - t0, 4), round(x, 5), round(est['v'], 4),
                              round(psi, 5), round(-est['omega'], 4),
                              round(a, 4), est['flags'], est['seq']])

            if agora - t_ult >= 1.0:
                t_ult = agora
                extra = ''
                if comp:
                    ag, pi = np.array(comp[-50:]).T
                    iguais = 100.0 * np.mean(np.sign(ag) == np.sign(pi))
                    extra = f' | OBSERVA: PID {pi.mean():+5.2f} agente {ag.mean():+5.2f}, mesmo sinal {iguais:3.0f}%'
                print(f'  {n:6d} passos | psi {np.mean(psis[-50:]):5.2f}deg | '
                      f'x {x*1000:+6.1f}mm | a {a:+6.2f} | '
                      f'perdidos {p.perdidos} | rede {np.mean(lat[-50:]):.2f}ms{extra}')

            if args.segundos and (agora - t0) >= args.segundos:
                break

    except KeyboardInterrupt:
        print('\nEncerrando...')
    finally:
        try:
            p.enviar_acao(0.0, 0, CMD_PARAR)
            time.sleep(0.05)
            p.texto('s')
            time.sleep(0.05)
            p.ser.close()
        except Exception:
            pass
        if esc:
            f.close()

    dur = time.time() - t0
    print(f'\n{n} passos em {dur:.1f} s  ({n/max(dur,1e-9):.1f} Hz)')
    if psis:
        print(f'|psi| medio = {np.mean(psis):.2f} graus   maximo = {np.max(psis):.2f}')
    print(f'pacotes perdidos = {p.perdidos}')
    if lat:
        print(f'latencia da rede: media {np.mean(lat):.2f} ms, pior {np.max(lat):.2f} ms')
    if comp:
        ag, pi = np.array(comp).T
        iguais = 100.0 * np.mean(np.sign(ag) == np.sign(pi))
        corr = float(np.corrcoef(ag, pi)[0, 1]) if len(ag) > 2 else float('nan')
        print()
        print('=== MODO OBSERVACAO: agente x PID no mesmo estado ===')
        print(f'  mesmo sinal em {iguais:.0f}% das amostras   correlacao {corr:+.3f}')
        print(f'  acao media: PID {pi.mean():+.3f}  agente {ag.mean():+.3f} m/s2')
        if iguais < 60 or corr < 0.3:
            print('  >> ALERTA: as decisoes discordam. Suspeite de sinal invertido')
            print('     em ENC_SINAL, em psi, ou na direcao do motor.')
            print('     NAO passe para a etapa seguinte antes de resolver.')
        else:
            print('  >> Coerente. O agente decide no mesmo sentido que o PID.')
    if args.log:
        print(f'log em {args.log}')


if __name__ == '__main__':
    main()
