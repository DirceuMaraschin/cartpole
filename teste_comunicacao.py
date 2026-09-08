import serial
import struct
import time

PORTA_MAC = '/dev/cu.usbserial-0001'
BAUD_RATE = 921600
FORMATO_SUBIDA = "<HBiiffBB"
FORMATO_DESCIDA = "<HBBBB"

def calcular_crc8(dados):
    crc = 0
    for byte in dados:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80: crc = (crc << 1) ^ 0x07
            else: crc <<= 1
            crc &= 0xFF
    return crc

def main():
    print(f"Abrindo porta {PORTA_MAC}...")
    try:
        # Instancia a porta sem abrir imediatamente
        porta = serial.Serial()
        porta.port = PORTA_MAC
        porta.baudrate = BAUD_RATE
        porta.timeout = 0.02
        
        # Desativa os sinais que causam o reset físico na placa
        porta.dtr = False
        porta.rts = False
        
        # Agora abre a porta de forma segura
        porta.open()
        print("Conexão estabelecida! Aguardando o cabeçalho do ESP32...")
        print(porta.read_all())
    except Exception as e:
        print(f"Erro: {e}")
        return

    t_inicio = time.time()
    
    try:
        while True:
            # 1. Determina a ação de teste baseada no tempo (alterna a cada 2 seg)
            tempo_decorrido = time.time() - t_inicio
            acao_teste = 1 if int(tempo_decorrido / 2) % 2 == 0 else 0
            nome_acao = "DIREITA" if acao_teste == 1 else "ESQUERDA"

            # 2. Leitura do pacote de estado
            porta.reset_input_buffer()
            porta.read_until(b'\x55\xAA')
            dados = porta.read(19)
            
            if len(dados) == 19:
                pacote_completo = b'\x55\xAA' + dados
                
                if calcular_crc8(pacote_completo[:-1]) == pacote_completo[-1]:
                    _, seq, x_pulsos, theta_cont, x_dot, th_dot, flags, crc_recebido = struct.unpack(FORMATO_SUBIDA, pacote_completo)                    
                    x_metros = x_pulsos * 0.025 / 1000.0
                    
                    # Imprime o estado recebido
                    print(f"RX | x: {x_metros: .4f} m | Vel cmd: {x_dot: .3f} m/s | Ação TX: {nome_acao}")
                    
                    # 3. Envio da ação
                    pacote_acao = struct.pack("<HBBB", 0xA55A, seq, acao_teste, 0)
                    porta.write(pacote_acao + struct.pack("<B", calcular_crc8(pacote_acao)))

            time.sleep(0.015) # Pequena pausa para sincronia

    except KeyboardInterrupt:
        print("\nTeste finalizado.")
    finally:
        porta.close()

if __name__ == "__main__":
    main()