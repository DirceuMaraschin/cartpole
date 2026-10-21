import numpy as np, sys; sys.path.insert(0,'.')
import pendulo_env as E
from pendulo_env import PenduloInvertido, A_MAX, X_LIM, PSI_LIM, V_MAX

def avaliar(ganhos, hz, n_ep=12, semente=1):
    KP,KD,KX,KV = ganhos
    E.DT_CTRL = 1.0/hz; E.N_SUB = max(1,int(round(E.DT_CTRL/E.DT_FIS)))
    env = PenduloInvertido(dominio_aleatorio=False, max_passos=int(20*hz))
    env.rng = np.random.default_rng(semente)
    durs, psis = [], []
    for ep in range(n_ep):
        obs = env.reset(); p=[]
        for k in range(env.max_passos):
            a = np.clip(KP*obs[2]*PSI_LIM + KD*obs[3]*E.PSID_ESC
                        + KX*obs[0]*X_LIM + KV*obs[1]*V_MAX, -A_MAX, A_MAX)/A_MAX
            obs,r,caiu,trunc,info = env.step(a)
            p.append(abs(info['psi']))
            if caiu or trunc: break
        durs.append((k+1)*E.DT_CTRL); psis.append(np.degrees(np.mean(p)))
    viv = np.mean(np.array(durs) >= 19.9)
    return viv, np.mean(psis), np.mean(durs)

print('=== TETO DE DESEMPENHO A 50 Hz: busca de ganhos ===')
print('ganhos originais (1 kHz): KP=40 KD=5.5 KX=1.5 KV=17.5\n')
base = (40,5.5,1.5,17.5)
v,p,d = avaliar(base, 50); print(f'originais a 50 Hz : vive {v*100:3.0f}%  |psi|={p:5.2f} deg')

rng = np.random.default_rng(0)
melhor = (0,99,None)
for it in range(220):
    KP = rng.uniform(8, 60); KD = rng.uniform(1, 14)
    KX = rng.uniform(0.2, 6); KV = rng.uniform(1, 20)
    v,p,d = avaliar((KP,KD,KX,KV), 50, n_ep=6)
    if v >= 1.0 and p < melhor[1]:
        melhor = (v, p, (KP,KD,KX,KV))
v,p,g = melhor
print(f'melhores p/ 50 Hz : vive {v*100:3.0f}%  |psi|={p:5.2f} deg')
print(f'   KP={g[0]:.1f} KD={g[1]:.2f} KX={g[2]:.2f} KV={g[3]:.1f}')
v2,p2,_ = avaliar(g, 50, n_ep=20, semente=7)
print(f'   confirmando em 20 episodios novos: vive {v2*100:.0f}%  |psi|={p2:.2f} deg')
print()
v3,p3,_ = avaliar(base, 200, n_ep=12); print(f'referencia: originais a 200 Hz -> |psi|={p3:.2f} deg')
