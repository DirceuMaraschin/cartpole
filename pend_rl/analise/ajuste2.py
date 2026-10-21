import numpy as np
pk = np.load('analise/picos.npy'); tp, ap = pk[:,0], pk[:,1]
L = 0.22463; w0 = np.sqrt(9.81/L)

# taxa de queda da amplitude entre picos consecutivos
dA = np.diff(ap)/np.diff(tp)
Am = 0.5*(ap[:-1]+ap[1:])
m = (Am > 0.05) & (dA < 0)
# dA/dt = -(b/2)A - 2c/(pi w0)   -> regressao linear em A
coef = np.polyfit(Am[m], dA[m], 1)
b = -2*coef[0]
c = -coef[1]*np.pi*w0/2
r2 = np.corrcoef(Am[m], dA[m])[0,1]**2
print('=== ATRITO PELO ENVELOPE ===')
print(f'pontos usados: {m.sum()}   R2 da reta: {r2:.4f}')
print(f'b (viscoso) = {b:.5f} 1/s')
print(f'c (Coulomb) = {c:.5f} rad/s^2')
print(f'  contribuicao viscosa a 50 deg: {b/2*0.87:.5f} rad/s por s')
print(f'  contribuicao Coulomb (constante): {2*c/(np.pi*w0):.5f} rad/s por s')

# validacao: simular janelas curtas (5 s) a partir de varios pontos
import pandas as pd
d = pd.read_csv('logs/livre_01.csv')
ta, pa, oa = d.t_s.values, d.phi_rad.values, d.omega_rads.values
i0 = np.where(np.abs(pa) > 0.05)[0][0]
ta, pa, oa = ta[i0:]-ta[i0], pa[i0:], oa[i0:]
DT = 0.001
def sim(phi0, om0, tmax, L, b, c):
    n=int(tmax/DT)+1; ph,om = phi0,om0; P=np.empty(n)
    for k in range(n):
        P[k]=ph
        om += (-9.81/L*np.sin(ph) - b*om - c*np.tanh(om/0.02))*DT
        ph += om*DT
    return np.arange(n)*DT, P

print('\n=== VALIDACAO EM JANELAS DE 5 s ===')
tot=[]
for t_ini in [0.5, 10, 20, 30, 40]:
    k0 = np.argmin(np.abs(ta - t_ini))
    st, sp = sim(pa[k0], oa[k0], 5.0, L, b, c)
    real = np.interp(st, ta[k0:]-ta[k0], pa[k0:])
    e = np.degrees(np.sqrt(np.mean((sp-real)**2)))
    amp = np.degrees(np.abs(real).max())
    tot.append(e)
    print(f'  t={t_ini:4.1f}s  amplitude={amp:5.1f}deg  erro RMS = {e:.3f} deg')
print(f'  media = {np.mean(tot):.3f} deg')
np.save('analise/par_final.npy', np.array([L,b,c]))
