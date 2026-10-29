"""main() real contra o ESP32 falso, com um travamento de 150 ms injetado,
reproduzindo 29/10. Verifica (a) que a guarda impede bytes-virando-tecla e
(b) que o relatorio final acusa o ciclo estourado."""
import os, pty, sys, time, threading, builtins
import os as _os; sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
import rl_bridge, teste_auto

mestre, escravo = pty.openpty()
os.set_blocking(mestre, False)
esp = teste_auto.ESP32Falso(mestre, observa_inicial=False)
esp.start()

# Travamento disparado por TEMPO (nao por contagem: o aquecimento tambem
# passa por agir e consumiria o gatilho antes do laco comecar).
armado = {'v': False, 'usado': False}
orig = rl_bridge.PoliticaNumpy.agir
def agir_travando(self, obs):
    if armado['v'] and not armado['usado']:
        armado['usado'] = True
        print('  [teste] injetando travamento de 150 ms')
        time.sleep(0.150)
    return orig(self, obs)
# Precisa ser a PoliticaNumpy: o torch saiu do laco quente, e patchear
# AtorCritico.agir deixava o travamento sem efeito -- o teste passava vazio.
rl_bridge.PoliticaNumpy.agir = agir_travando

builtins.input = lambda *a, **k: ''
# Sob 'timeout' o stdin esta em EOF: a thread real enfileiraria 'q' na hora e
# main() encerraria antes de processar um pacote -- o teste passaria vazio.
rl_bridge.ler_teclado = lambda fila: time.sleep(3600)
def agenda():
    time.sleep(9.0);  armado['v'] = True      # ja dentro do laco
    time.sleep(3.0);  import _thread; _thread.interrupt_main()
threading.Thread(target=agenda, daemon=True).start()

sys.argv = ['rl_bridge.py', '--porta', os.ttyname(escravo), '--baud', '115200',
            '--modelo', 'modelos/melhor.pt', '--auto',
            '--log', 'logs/_travada.csv']
os.chdir(_os.path.dirname(_os.path.abspath(__file__)))
try:
    rl_bridge.main()
except KeyboardInterrupt:
    pass
esp.rodando = False
print()
print('=== VEREDITO ===')
print(f'bytes de acao lidos como TECLA apos sair do RL: {esp.texto_pos_rl}')
print(f'saidas por watchdog no firmware falso: {esp.saidas_por_watchdog}')
print('  (na bancada de 29/10 foram 21 bytes = Z + 18 "comando desconhecido")')
print('  OK' if esp.texto_pos_rl == 0 else '  FALHOU: a guarda nao segurou')