import serial
import struct
import time
import pandas as pd
import numpy as np

# HARDWARE E COMUNICAÇÃO
PORTA_SERIAL = '/dev/cu.usbserial-0001' # ls /dev/cu.*
BAUD_RATE = 921600
FORMATO_SUBIDA = "<HBiiffBB"  # 21 bytes
FORMATO_DESCIDA = "<HBBBB"    # 6 bytes

# GANHOS PID 
## V_ideal = (K_p * theta) + (K_d * theta_dot) + (K_x * x)
Kp = 15.0   # Ganho Proporcional - Ângulo
Kd = 2.5    # Ganho Derivativo - Velocidade Angular
Kx = -1.2   # Ganho Proporcional - Posição

def calcular_crc8(dados):
    # Cyclic Redundancy Check de 8 bits) -> código de detecção de erros para validação de pacote
    crc = 0
    for byte in dados:
        crc ^= byte
        for byte in dados:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
    return crc

def main():
    try:
        porta = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=0.02)
    except Exception as e:
        print(f"Erro ao abrir porta serial: {e}")
        return

    historico_estado = []
    print("Inicialdo validação PID (Ctrl+C to terminate)")

    t_inicio = time.time()

    try:
        while True:
            t_atual = time.time() - t_inicio

            # 1. Limpeza do buffer e sync
            porta.reset_input_buffer()
            porta.read_until(b'\x55\xAA')   # Aguarda cabeçalho 0xAA55
            dados = porta.read(19)          # Lê o restante do pacote

            if len(dados) == 19:
                pacote_completo = b'\x55\xAA' + dados
                *estado_bruto, crc_recebido = struct.unpack(FORMATO_SUBIDA, pacote_completo)

                ## Validação de integridade
                if crc_recebido != calcular_crc8(pacote_completo[:-1]):
                    continue

                _, seq, x_pulsos, theta_cont, x_dot, th_dot, flags = estado_bruto

                # 2. Conversão da cinemática
                x_metros = x_pulsos * 0.000025
                theta_rad = theta_cont * 0.00261799

                # 3. Lógica de controle PID
                v_ideal = (Kp * theta_rad) + (Kd * th_dot) + (Kx * x_metros)

                if v_ideal > 0.1:
                    acao = 1    # incremento -> p/ direita
                elif v_ideal < -0.1:
                    acao = 0    # incremento -> p/ esquerda
                else:
                    acao = 2    # manter velocidade

                # 4. Transmitir ação
                pacote_acao = struct.pack("<HBBB", 0xA55A, seq, acao, 0)
                crc_acao = calcular_crc8(pacote_acao)
                porta.write(pacote_acao + struct.pack("<B", crc_acao))

                # 5. Coleta da matriz
                historico_estado.append([t_atual, x_metros, theta_rad, th_dot, acao])

            # mantém o laço de 20ms
            time.sleep(max(0, 0.02 - (time.time() - t_inicio - t_atual)))

    except KeyboardInterrupt:
        print("\nColeta finalizada. Processando estatísticas de estabilidade...")

    finally:
        porta.close()
        processar_dados(historico_estado)

def processar_dados(historico):
    if not historico:
        print("Nenhum dado coletado..")
        return

    df = pd.DataFrame(historico, columns=['Tempo', 'Posicao_X', 'Angulo_Theta', 'Velocidade_Angular', 'Acao_Discreta'])

    # Exportação bruta
    df.to_csv('matriz_validacao_pid.csv', index=False)
    
    # Segmentação e Análise Estatística
    print("\n--- MÉTRICAS DE ESTABILIDADE (ÂNGULO THETA) ---")
    print(f"Desvio Padrão: {df['Angulo_Theta'].std():.5f} rad")
    
    # Cálculo de Quintis usando Pandas
    quintis = df['Angulo_Theta'].quantile([0.2, 0.4, 0.5, 0.6, 0.8]).to_dict()
    print("\nAnálise de Quintis (radianos):")
    print(f"1º Quintil (20%): {quintis[0.2]:.5f}")
    print(f"2º Quintil (40%): {quintis[0.4]:.5f}")
    print(f"Mediana    (50%): {quintis[0.5]:.5f} -> O ideal é estar em 0.00")
    print(f"4º Quintil (60%): {quintis[0.6]:.5f}")
    print(f"5º Quintil (80%): {quintis[0.8]:.5f}")
    
    print(f"\nArquivo 'matriz_validacao_pid.csv' exportado com sucesso.")

if __name__ == "__main__":
    main()