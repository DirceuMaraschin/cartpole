"""Testa a sequencia do agente COMPLETO contra o ESP32 falso."""
import os, pty, sys, time, builtins, threading

# Caminho do PROPRIO arquivo, nao um absoluto fixo: a primeira versao tinha o
# caminho do container embutido e quebrou na primeira execucao na sua maquina.
_AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _AQUI)
os.chdir(_AQUI)
import rl_bridge, teste_auto

mestre, escravo = pty.openpty(); os.set_blocking(mestre, False)
esp = teste_auto.ESP32Falso(mestre); esp.zerado = False; esp.start()
builtins.input = lambda *a, **k: ''
p = None
for baud in (115200, 9600):
    try: p = rl_bridge.Ponte(os.ttyname(escravo), baud); break
    except OSError: continue
p.ler_constantes(espera=1.0)
ok = rl_bridge.sequencia_completa(p)
# responde por 1 s e confere a flag
n, comp = 0, 0
t0 = time.time()
while time.time() - t0 < 1.0:
    est = p.ler_estado(0.05)
    if est:
        p.enviar_acao(0.5, est['seq']); n += 1
        if est['flags'] & rl_bridge.F_COMPLETO: comp += 1
esp.rodando = False; p.ser.close(); os.close(mestre); os.close(escravo)
print()
print('=== VEREDITO ===')
print(f'sequencia completou: {ok}')
print(f'pacotes em 1 s: {n} (esperado ~50)')
print(f'com F_COMPLETO aceso: {comp}/{n}')
print(f'observacao no g: {esp.observa_no_r} (deve ser False)')
print(f'saidas por watchdog: {esp.saidas_por_watchdog} (deve ser 0)')
bom = ok and n >= 40 and comp == n and not esp.observa_no_r and esp.saidas_por_watchdog == 0
print('  OK' if bom else '  FALHOU')
sys.exit(0 if bom else 1)
