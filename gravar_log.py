#!/usr/bin/env python3
"""
Grava o log CSV do pendulo invertido (ESP32) em arquivo.

Uso:
    python gravar_log.py --porta COM5 --saida logs/pid_01.csv
    python gravar_log.py --porta /dev/ttyUSB0 --saida logs/pid_01.csv --segundos 120

O script NAO liga o log sozinho por engano: ele envia o comando 'l' apenas se
voce passar --auto. Por padrao ele so escuta, e voce controla pelo teclado.

Teclas enquanto grava (digite e ENTER):
    l  -> liga/desliga o log no ESP32
    i  -> swing-up + PID        e -> captura manual
    s  -> parar                 c -> ir ao centro
    z  -> definir zero          p -> posicao
    k  -> reimprimir constantes
    q  -> encerrar a gravacao aqui no PC
"""

import argparse
import csv
import os
import queue
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("Falta a biblioteca pyserial. Instale com:  pip install pyserial")


COLUNAS = ["t_s", "x_m", "v_ms", "phi_rad", "omega_rads", "a_ms2", "modo"]


def ler_teclado(fila):
    """Le comandos do teclado em uma thread separada, sem travar a leitura serial."""
    for linha in sys.stdin:
        fila.put(linha.strip())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--porta", required=True,
                    help="porta serial: COM5, /dev/ttyUSB0, /dev/cu.usbserial-0001 ...")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--saida", default="log_pendulo.csv")
    ap.add_argument("--segundos", type=float, default=0,
                    help="encerra sozinho apos N segundos (0 = sem limite)")
    ap.add_argument("--auto", action="store_true",
                    help="envia 'l' ao conectar, ligando o log imediatamente")
    args = ap.parse_args()

    pasta = os.path.dirname(os.path.abspath(args.saida))
    os.makedirs(pasta, exist_ok=True)
    caminho_meta = os.path.splitext(args.saida)[0] + "_constantes.txt"

    print(f"Abrindo {args.porta} a {args.baud} baud...")
    ser = serial.Serial(args.porta, args.baud, timeout=1)
    time.sleep(2.0)          # o ESP32 reinicia ao abrir a porta
    ser.reset_input_buffer()

    fila = queue.Queue()
    threading.Thread(target=ler_teclado, args=(fila,), daemon=True).start()

    if args.auto:
        ser.write(b"l")
        print("Comando 'l' enviado: log ligado.")

    arq = open(args.saida, "w", newline="", encoding="utf-8")
    escritor = csv.writer(arq)
    escritor.writerow(COLUNAS)

    meta = open(caminho_meta, "w", encoding="utf-8")

    n_linhas = 0
    n_ruins = 0
    t_inicio = time.time()
    ultimo_aviso = 0.0

    print("Gravando. Digite 'l' + ENTER para ligar/desligar o log, 'q' para sair.\n")

    try:
        while True:
            # --- comandos do teclado -> ESP32 ---
            while not fila.empty():
                cmd = fila.get()
                if cmd.lower() == "q":
                    raise KeyboardInterrupt
                if cmd:
                    ser.write(cmd[0].encode())
                    print(f"  [enviado: {cmd[0]}]")

            # --- leitura da serial ---
            bruto = ser.readline()
            if not bruto:
                continue
            linha = bruto.decode("utf-8", errors="replace").strip()
            if not linha:
                continue

            # comentarios e mensagens do firmware comecam com '#'
            if linha.startswith("#"):
                print(linha)
                meta.write(linha + "\n")
                meta.flush()
                continue

            # cabecalho do CSV reenviado pelo firmware
            if linha.startswith("t_s,"):
                continue

            partes = linha.split(",")
            if len(partes) != len(COLUNAS):
                n_ruins += 1
                continue
            try:
                valores = [float(p) for p in partes[:-1]] + [int(partes[-1])]
            except ValueError:
                n_ruins += 1
                continue

            escritor.writerow(valores)
            n_linhas += 1

            agora = time.time()
            if agora - ultimo_aviso >= 2.0:
                ultimo_aviso = agora
                dur = valores[0]
                print(f"  {n_linhas} amostras | t={dur:7.2f} s | "
                      f"x={valores[1]*1000:6.1f} mm | ang={valores[3]*57.2958:7.1f} deg "
                      f"| modo={valores[6]}")

            if args.segundos and (agora - t_inicio) >= args.segundos:
                break

    except KeyboardInterrupt:
        print("\nEncerrando...")
    finally:
        try:
            ser.write(b"s")      # para o carrinho por seguranca
            time.sleep(0.1)
            ser.close()
        except Exception:
            pass
        arq.close()
        meta.close()

    print(f"\n{n_linhas} amostras gravadas em {args.saida}")
    if n_ruins:
        print(f"{n_ruins} linhas descartadas (ruido na serial, normal se poucas)")
    print(f"Constantes do sistema em {caminho_meta}")


if __name__ == "__main__":
    main()
