import pandas as pd, numpy as np
d = pd.read_csv('logs/livre_01.csv')
ta, pa = d.t_s.values, d.phi_rad.values
i0 = np.where(np.abs(pa) > 0.05)[0][0]
ta, pa = ta[i0:]-ta[i0], pa[i0:]
# omega por diferenca central no proprio phi (sem o atraso do filtro do firmware)
om_cd = np.gradient(pa, ta)
L, b, c = np.load('analise/par_final.npy')
DT = 0.001
def sim(phi0, om0, tmax):
    n=int(tmax/DT)+1; ph,om=phi0,om0; P=np.empty(n)
    for k in range(n):
        P[k]=ph
        om += (-9.81/L*np.sin(ph) - b*om - c*np.tanh(om/0.02))*DT
        ph += om*DT
    return np.arange(n)*DT, P
print(f'parametros: L={L*1000:.2f}mm  b={b:.5f}  c={c:.5f}')
print('\n=== VALIDACAO (omega inicial por diferenca central) ===')
errs=[]
for t_ini in [0.5, 5, 10, 15, 20, 25, 30, 35, 40, 45]:
    k0 = np.argmin(np.abs(ta - t_ini))
    st, sp = sim(pa[k0], om_cd[k0], 5.0)
    real = np.interp(st, ta[k0:]-ta[k0], pa[k0:])
    e = np.degrees(np.sqrt(np.mean((sp-real)**2)))
    errs.append(e)
    print(f'  t={t_ini:4.1f}s  amp={np.degrees(np.abs(real).max()):5.1f}deg  erro RMS={e:6.3f} deg')
print(f'  MEDIA = {np.mean(errs):.3f} deg   PIOR = {np.max(errs):.3f} deg')
# guarda uma janela longa para o grafico
k0 = np.argmin(np.abs(ta - 10.0))
st, sp = sim(pa[k0], om_cd[k0], 12.0)
real = np.interp(st, ta[k0:]-ta[k0], pa[k0:])
np.save('analise/graf_valid.npy', np.c_[st, real, sp])
