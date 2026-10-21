import pandas as pd, numpy as np
d = pd.read_csv('logs/pid_01.csv')
d = d[d.modo == 4].reset_index(drop=True)      # so o trecho equilibrando
t = d.t_s.values; x=d.x_m.values; v=d.v_ms.values
phi=d.phi_rad.values; om=d.omega_rads.values; a=d.a_ms2.values
psi = np.arctan2(np.sin(np.pi-phi), np.cos(np.pi-phi))
psid = -om
print(f'=== TRECHO EQUILIBRANDO: {len(d)} amostras, {t[-1]-t[0]:.1f} s ===')
print(f'psi : media={np.degrees(psi.mean()):+.3f}deg  desvio={np.degrees(psi.std()):.3f}deg  pico={np.degrees(np.abs(psi).max()):.2f}deg')
print(f'x   : media={x.mean()*1000:+.1f}mm  desvio={x.std()*1000:.1f}mm  pico={np.abs(x).max()*1000:.0f}mm')
print(f'a   : desvio={a.std():.3f} m/s2   saturou em {100*np.mean(np.abs(a)>=7.99):.1f}% do tempo')

# --- confere se a acao do log bate com o PID reconstruido ---
KP,KD,KX,KV = 40.0,5.5,1.5,17.5
base = KP*psi + KD*psid + KX*x + KV*v
res = a - np.clip(base, -8, 8)
print(f'\n=== CONFERENCIA DO PID (sem o termo integral) ===')
print(f'residuo a - PID: media={res.mean():+.4f}  desvio={res.std():.4f} m/s2')
print(f'  (o residuo deve ser o termo integral, limitado a +-0.5)')
print(f'  residuo dentro de +-0.5: {100*np.mean(np.abs(res)<=0.5):.1f}% das amostras')

# --- quantizacao do encoder ---
passo_enc = 2*np.pi/2400
dphi = np.diff(phi)
nz = dphi[np.abs(dphi) > 1e-9]
print(f'\n=== ENCODER ===')
print(f'passo teorico = {np.degrees(passo_enc):.4f} deg')
print(f'menor salto observado = {np.degrees(np.abs(nz).min()):.4f} deg')
print(f'saltos multiplos do passo: {100*np.mean(np.abs(np.round(nz/passo_enc)-nz/passo_enc)<0.01):.1f}%')

# --- ruido de omega: compara filtrado do firmware com diferenca central ---
om_cd = np.gradient(phi, t)
dif = om - om_cd
print(f'\n=== OMEGA ===')
print(f'desvio do omega filtrado      = {om.std():.4f} rad/s')
print(f'desvio da diferenca central   = {om_cd.std():.4f} rad/s')
print(f'desvio da diferenca (filtro)  = {dif.std():.4f} rad/s')

# --- atraso efetivo: correlacao cruzada entre acao e aceleracao angular resultante ---
psidd = np.gradient(np.gradient(psi, t), t)
A = (a - a.mean())/a.std(); B = (psidd - psidd.mean())/psidd.std()
lags = np.arange(-20, 21)
cc = [np.corrcoef(A[20:-20], B[20+l:len(B)-20+l])[0,1] for l in lags]
cc = np.array(cc)
# o acoplamento teorico e psidd = (g sin psi - a cos psi)/L -> a entra com sinal negativo
imin = np.argmin(cc)
print(f'\n=== ATRASO EFETIVO DA MALHA ===')
print(f'correlacao mais negativa no lag = {lags[imin]} amostras = {lags[imin]*10:.0f} ms  (r={cc[imin]:.3f})')
for l in range(-4,5):
    print(f'   lag {l:+d} ({l*10:+3d} ms): r = {cc[lags==l][0]:+.3f}')
np.save('analise/pid_trecho.npy', np.c_[t,x,v,psi,psid,a])
