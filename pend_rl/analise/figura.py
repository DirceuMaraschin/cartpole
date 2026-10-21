import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd
plt.rcParams.update({'font.size':9,'axes.grid':True,'grid.alpha':0.3,'figure.dpi':130})
fig, ax = plt.subplots(3,1, figsize=(9.5,8.2))

g = np.load('analise/graf_valid.npy')
ax[0].plot(g[:,0], np.degrees(g[:,1]), lw=2.2, color='#1f3f66', label='real (encoder)')
ax[0].plot(g[:,0], np.degrees(g[:,2]), lw=1.1, color='#d04a2a', ls='--', label='simulado')
ax[0].set_title('Validação 1 — oscilação livre, 12 s (12 ciclos) · erro RMS 0,85°', loc='left', fontweight='bold')
ax[0].set_ylabel('ângulo φ (graus)'); ax[0].legend(loc='upper right', ncol=2)

d = pd.read_csv('logs/swing_01.csv'); sw=d[d.modo==3]
seg=d.loc[sw.index[0]:sw.index[-1]].reset_index(drop=True)
t=seg.t_s.values-seg.t_s.values[0]; phi=seg.phi_rad.values
v=seg.v_ms.values; ar=np.gradient(v,t); om=np.gradient(phi,t)
L,b,c=np.load('analise/par_final.npy'); DT=0.001
def sim(p0,o0,ta,aa,tmax):
    n=int(tmax/DT)+1; ph,o=p0,o0; P=np.empty(n)
    for k in range(n):
        tt=k*DT; P[k]=ph; a=np.interp(tt,ta,aa)
        o += ((-9.81*np.sin(ph)-a*np.cos(ph))/L - b*o - c*np.tanh(o/0.02))*DT; ph+=o*DT
    return np.arange(n)*DT,P
k0=np.argmin(np.abs(t-1.0)); st,sp=sim(phi[k0],om[k0],t[k0:]-t[k0],ar[k0:],1.8)
real=np.interp(st,t[k0:]-t[k0],phi[k0:])
ax[1].plot(st,np.degrees(real),lw=2.2,color='#1f3f66',label='real')
ax[1].plot(st,np.degrees(sp),lw=1.1,color='#d04a2a',ls='--',label='simulado')
ax[1].set_title('Validação 2 — swing-up com o carrinho acelerando · excursão 190°, erro RMS 0,6°', loc='left', fontweight='bold')
ax[1].set_ylabel('ângulo φ (graus)'); ax[1].legend(loc='upper right', ncol=2)

h=[0.1,0.2,0.3,0.5,1.0]; e=[0.30,0.70,1.28,3.90,54.23]
ax[2].semilogy(h,e,'o-',color='#1f3f66',lw=2,ms=7, label='erro medido')
ax[2].semilogy(h, 0.30*np.exp((np.array(h)-0.1)/0.151), '--', color='#8a6d1f', lw=1.4,
               label=r'crescimento teórico $e^{t/\tau}$, $\tau$=0,151 s')
ax[2].axvline(0.02, color='#2f6b4f', lw=1.6)
ax[2].annotate('passo do agente\n20 ms', xy=(0.021,1.2), fontsize=8, color='#2f6b4f')
ax[2].set_title('Validação 3 — perto do topo o erro cresce exponencialmente (isso é a física, não o modelo)', loc='left', fontweight='bold')
ax[2].set_xlabel('horizonte de previsão (s)'); ax[2].set_ylabel('erro RMS (graus)')
ax[2].legend(loc='lower right'); ax[2].set_xlim(0,1.05)
plt.tight_layout(); plt.savefig('analise/validacao_modelo.png', bbox_inches='tight')
print('figura salva')
