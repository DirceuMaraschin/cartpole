import pandas as pd, numpy as np
L,b,c = np.load('analise/par_final.npy')
DT = 0.001
def sim_acoplado(phi0, om0, t_a, a_cmd, tmax):
    """integra phi com a aceleracao REAL do carrinho vinda do log"""
    n=int(tmax/DT)+1; ph,om=phi0,om0; P=np.empty(n)
    for k in range(n):
        tt=k*DT; P[k]=ph
        a = np.interp(tt, t_a, a_cmd)
        acc = (-9.81*np.sin(ph) - a*np.cos(ph))/L - b*om - c*np.tanh(om/0.02)
        om += acc*DT; ph += om*DT
    return np.arange(n)*DT, P

print('=== VALIDACAO COM O CARRINHO EM MOVIMENTO (swing-up) ===')
print('modelo: phi_dd = (-g sin(phi) - a cos(phi))/L - b*om - c*sign(om)\n')
for arq in ['swing_01','swing_02']:
    d = pd.read_csv(f'logs/{arq}.csv')
    sw = d[d.modo==3]
    if len(sw)<100: continue
    i0,i1 = sw.index[0], sw.index[-1]
    seg = d.loc[i0:i1].reset_index(drop=True)
    t = seg.t_s.values - seg.t_s.values[0]
    phi, a = seg.phi_rad.values, seg.a_ms2.values
    om_cd = np.gradient(phi, t)
    print(f'{arq}: swing-up de {t[-1]:.1f}s, |a| ate {np.abs(a).max():.2f} m/s2')
    for jan, ini in [(2.0,0.2),(2.0,2.0),(3.0,1.0)]:
        k0 = np.argmin(np.abs(t-ini))
        if t[-1]-t[k0] < jan: continue
        st,sp = sim_acoplado(phi[k0], om_cd[k0], t[k0:]-t[k0], a[k0:], jan)
        real = np.interp(st, t[k0:]-t[k0], phi[k0:])
        e = np.degrees(np.sqrt(np.mean((sp-real)**2)))
        print(f'   janela {jan:.0f}s a partir de t={ini:.1f}s: erro RMS = {e:6.2f} deg  (excursao {np.degrees(real.max()-real.min()):.0f} deg)')
    print()
