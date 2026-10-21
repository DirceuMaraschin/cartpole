import pandas as pd, numpy as np
from scipy.optimize import minimize

d = pd.read_csv('logs/livre_01.csv')
t_all, phi_all = d.t_s.values, d.phi_rad.values
i0 = np.where(np.abs(phi_all) > 0.05)[0][0]
t = t_all[i0:] - t_all[i0]; phi_r = phi_all[i0:]
# recorta antes do ruido final (amplitude < 2 graus)
fim = np.where(np.abs(phi_r) > 0.035)[0][-1]
t, phi_r = t[:fim], phi_r[:fim]
DT = 0.001   # integra a 1 kHz, igual ao firmware

def simular(L, b, c, phi0, om0, tmax):
    n = int(tmax/DT)+1
    ph, om = phi0, om0
    out_t = np.empty(n); out_p = np.empty(n)
    g_L = 9.81/L
    for k in range(n):
        out_t[k] = k*DT; out_p[k] = ph
        acc = -g_L*np.sin(ph) - b*om - c*np.tanh(om/0.02)
        om += acc*DT
        ph += om*DT
    return out_t, out_p

def erro(p):
    L, b, c, om0 = p
    if L<=0 or b<0 or c<0: return 1e9
    st, sp = simular(L, b, c, phi_r[0], om0, t[-1])
    sim = np.interp(t, st, sp)
    return np.sqrt(np.mean((sim-phi_r)**2))

melhor = None
for L0 in [0.2246]:
    for b0 in [0.02, 0.06]:
        for c0 in [0.02, 0.08]:
            r = minimize(erro, [L0,b0,c0,0.0], method='Nelder-Mead',
                         options={'maxiter':3000,'xatol':1e-6,'fatol':1e-7})
            if melhor is None or r.fun < melhor.fun: melhor = r

L,b,c,om0 = melhor.x
print('=== AJUSTE SIMULTANEO (72 s de oscilacao livre) ===')
print(f'L_eq      = {L*1000:.2f} mm')
print(f'b (viscoso)  = {b:.5f} 1/s')
print(f'c (Coulomb)  = {c:.5f} rad/s^2')
print(f'omega inicial= {om0:+.4f} rad/s')
print(f'RMS do erro  = {melhor.fun*1000:.3f} mrad = {np.degrees(melhor.fun):.3f} deg')
print(f'w0 = {np.sqrt(9.81/L):.4f} rad/s   T0 = {2*np.pi/np.sqrt(9.81/L):.4f} s')

st, sp = simular(L,b,c,phi_r[0],om0,t[-1])
sim = np.interp(t, st, sp)
np.save('analise/valid_livre.npy', np.c_[t, phi_r, sim])
# erro por trecho
for lo,hi in [(0,10),(10,25),(25,40),(40,55)]:
    m = (t>=lo)&(t<hi)
    if m.sum(): print(f'  erro RMS {lo:2d}-{hi:2d}s: {np.degrees(np.sqrt(np.mean((sim[m]-phi_r[m])**2))):.3f} deg')
np.save('analise/params.npy', melhor.x)
