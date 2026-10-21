import numpy as np, sys; sys.path.insert(0,'.')
import pendulo_env as E
from pendulo_env import PenduloInvertido, PIDFirmware

def testar(hz, n_ep=20, rand=False):
    E.DT_CTRL = 1.0/hz
    E.N_SUB = max(1, int(round(E.DT_CTRL/E.DT_FIS)))
    env = PenduloInvertido(dominio_aleatorio=rand, max_passos=int(20*hz))
    env.rng = np.random.default_rng(0)
    pid = PIDFirmware()
    dur, psis, xs = [], [], []
    for ep in range(n_ep):
        obs = env.reset(); pid.reset(); p,xx = [],[]
        for k in range(env.max_passos):
            obs, r, caiu, trunc, info = env.step(pid(obs, E.DT_CTRL))
            p.append(abs(info['psi'])); xx.append(abs(info['x']))
            if caiu or trunc: break
        dur.append((k+1)*E.DT_CTRL); psis.append(np.degrees(np.mean(p))); xs.append(np.mean(xx)*1000)
    return np.mean(dur), np.mean(psis), np.mean(xs), np.sum(np.array(dur)>=19.9)

print('=== EFEITO DA TAXA DE CONTROLE (modelo nominal, PID do firmware) ===')
print(f'{"taxa":>8} {"sobreviveu":>12} {"|psi| medio":>12} {"|x| medio":>11}')
for hz in [1000, 500, 200, 100, 50, 25]:
    d,p,x,s = testar(hz)
    print(f'{hz:6d}Hz {s:9d}/20 {p:10.2f} deg {x:8.0f} mm')
print('\n   bancada real (PID a 1 kHz): |psi| = 1,90 deg')
