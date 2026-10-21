import pandas as pd, numpy as np
L,b,c = np.load('analise/par_final.npy'); DT=0.001
def sim_ac(phi0, om0, t_a, a_real, tmax):
    n=int(tmax/DT)+1; ph,om=phi0,om0; P=np.empty(n)
    for k in range(n):
        tt=k*DT; P[k]=ph
        a=np.interp(tt,t_a,a_real)
        om += ((-9.81*np.sin(ph) - a*np.cos(ph))/L - b*om - c*np.tanh(om/0.02))*DT
        ph += om*DT
    return np.arange(n)*DT,P

print('=== ACELERACAO REAL = dv/dt (nao a comandada) ===\n')
for arq in ['swing_01','swing_02']:
    d = pd.read_csv(f'logs/{arq}.csv'); sw = d[d.modo==3]
    seg = d.loc[sw.index[0]:sw.index[-1]].reset_index(drop=True)
    t = seg.t_s.values - seg.t_s.values[0]
    phi, v, acmd = seg.phi_rad.values, seg.v_ms.values, seg.a_ms2.values
    a_real = np.gradient(v, t)
    om_cd = np.gradient(phi, t)
    sat = np.mean(np.abs(v) >= 0.4499)
    print(f'{arq}: velocidade saturada em {100*sat:.0f}% do swing-up')
    print(f'   a comandada: desvio {acmd.std():.2f}   a real (dv/dt): desvio {a_real.std():.2f} m/s2')
    errs=[]
    for ini in [0.3, 1.0, 1.8, 2.6, 3.4]:
        if t[-1]-ini < 1.2: continue
        k0=np.argmin(np.abs(t-ini))
        st,sp = sim_ac(phi[k0], om_cd[k0], t[k0:]-t[k0], a_real[k0:], 1.2)
        real = np.interp(st, t[k0:]-t[k0], phi[k0:])
        e=np.degrees(np.sqrt(np.mean((sp-real)**2))); errs.append(e)
        print(f'   t={ini:.1f}s +1.2s: erro RMS={e:6.2f} deg  (excursao {np.degrees(real.max()-real.min()):5.0f} deg)')
    print(f'   media={np.mean(errs):.2f} deg\n')

# e no trecho de EQUILIBRIO do pid_01, onde v nao satura
print('=== TRECHO DE EQUILIBRIO (pid_01), onde a velocidade quase nao satura ===')
d = pd.read_csv('logs/pid_01.csv'); eq = d[d.modo==4]
seg = d.loc[eq.index[0]:eq.index[-1]].reset_index(drop=True)
t=seg.t_s.values-seg.t_s.values[0]; phi=seg.phi_rad.values
v=seg.v_ms.values; a_real=np.gradient(v,t); om_cd=np.gradient(phi,t)
print(f'   velocidade saturada em {100*np.mean(np.abs(v)>=0.4499):.1f}% do tempo')
errs=[]
for ini in [2,6,10,14,18,22,26,30]:
    k0=np.argmin(np.abs(t-ini))
    st,sp=sim_ac(phi[k0],om_cd[k0],t[k0:]-t[k0],a_real[k0:],2.0)
    real=np.interp(st,t[k0:]-t[k0],phi[k0:])
    e=np.degrees(np.sqrt(np.mean((sp-real)**2))); errs.append(e)
    print(f'   t={ini:2d}s +2s: erro RMS={e:5.2f} deg')
print(f'   MEDIA={np.mean(errs):.2f} deg')
