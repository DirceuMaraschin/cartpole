import numpy as np, sys
sys.path.insert(0, '.')
from pendulo_env import PenduloInvertido, PIDFirmware, PSI_LIM, X_LIM

def rodar(env, n_ep=30, semente=0):
    env.rng = np.random.default_rng(semente)
    pid = PIDFirmware()
    dur, psis, xs, quedas = [], [], [], 0
    for ep in range(n_ep):
        obs = env.reset(); pid.reset()
        p_ep, x_ep = [], []
        for k in range(env.max_passos):
            a = pid(obs)
            obs, r, caiu, trunc, info = env.step(a)
            p_ep.append(abs(info['psi'])); x_ep.append(abs(info['x']))
            if caiu: quedas += 1; break
            if trunc: break
        dur.append((k+1)*0.020)
        psis.append(np.degrees(np.mean(p_ep))); xs.append(np.mean(x_ep)*1000)
    return np.array(dur), np.array(psis), np.array(xs), quedas

print('=== PORTAO DE QUALIDADE: o PID do firmware dentro do simulador ===')
print('(30 episodios de ate 20 s cada)\n')

for nome, kw in [('modelo nominal (sem randomizacao)', dict(dominio_aleatorio=False)),
                 ('com randomizacao de dominio',       dict(dominio_aleatorio=True))]:
    env = PenduloInvertido(max_passos=1000, **kw)
    dur, psi, x, q = rodar(env)
    print(f'{nome}:')
    print(f'   sobreviveu 20 s em {np.sum(dur>=19.99)}/30 episodios   (quedas: {q})')
    print(f'   duracao media = {dur.mean():.2f} s')
    print(f'   |psi| medio   = {psi.mean():.2f} deg   (real na bancada: 1,90 deg)')
    print(f'   |x| medio     = {x.mean():.0f} mm      (real na bancada: 175 mm)')
    print()

# comparacao direta com o log real
print('=== REFERENCIA: o mesmo PID no equipamento real (pid_01.csv) ===')
print('   equilibrou 37,9 s ate perder')
print('   |psi| desvio = 1,90 deg   |x| medio = 175 mm (derivou ate a parede)')
