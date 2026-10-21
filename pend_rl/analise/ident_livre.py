import pandas as pd, numpy as np
from scipy.special import ellipk

d = pd.read_csv('logs/livre_01.csv')
t, phi = d.t_s.values, d.phi_rad.values

i0 = np.where(np.abs(phi) > 0.05)[0][0]
t, phi = t[i0:], phi[i0:]
t = t - t[0]

# --- picos (amplitude de cada meio-ciclo) e cruzamentos de zero ---
picos_t, picos_a = [], []
for k in range(1, len(phi)-1):
    if (phi[k] >= phi[k-1] and phi[k] > phi[k+1]) or (phi[k] <= phi[k-1] and phi[k] < phi[k+1]):
        if abs(phi[k]) > 0.02:
            picos_t.append(t[k]); picos_a.append(abs(phi[k]))
picos_t, picos_a = np.array(picos_t), np.array(picos_a)
# remove repetidos do patamar de quantizacao
keep = np.r_[True, np.diff(picos_t) > 0.2]
picos_t, picos_a = picos_t[keep], picos_a[keep]

cruz = []
for k in range(1, len(phi)):
    if phi[k-1] < 0 <= phi[k]:
        f = -phi[k-1]/(phi[k]-phi[k-1]); cruz.append(t[k-1]+f*(t[k]-t[k-1]))
cruz = np.array(cruz)
per  = np.diff(cruz)
tper = 0.5*(cruz[:-1]+cruz[1:])

# amplitude media durante cada periodo
amp_per = np.array([picos_a[(picos_t>=cruz[i])&(picos_t<cruz[i+1])].mean()
                    if ((picos_t>=cruz[i])&(picos_t<cruz[i+1])).any() else np.nan
                    for i in range(len(per))])
ok = (~np.isnan(amp_per)) & (amp_per > 0.06) & (per > 0.8) & (per < 1.2)
per_ok, amp_ok = per[ok], amp_per[ok]

# T(a) = T0 * (2/pi) * K(sin(a/2))  -> extrai T0 de cada par e faz a mediana
fator = (2/np.pi)*ellipk(np.sin(amp_ok/2)**2)
T0_est = per_ok/fator
T0 = np.median(T0_est)
w0 = 2*np.pi/T0
L  = 9.81/w0**2

print('=== PERIODO E COMPRIMENTO EQUIVALENTE ===')
print(f'pares periodo/amplitude usados : {ok.sum()}')
print(f'periodo bruto: primeiro={per_ok[0]:.4f}s  ultimo={per_ok[-1]:.4f}s')
print(f'amplitude    : primeira={np.degrees(amp_ok[0]):.1f}deg  ultima={np.degrees(amp_ok[-1]):.1f}deg')
print(f'T0 (amplitude->0) = {T0:.4f} s   (desvio {T0_est.std()*1000:.1f} ms)')
print(f'w0 = {w0:.4f} rad/s')
print(f'L_eq MEDIDO  = {L*1000:.1f} mm')
print(f'L_eq firmware= 220.0 mm   -> erro de {100*(L-0.220)/0.220:+.1f}%')

# --- modelo de atrito: envelope de decaimento ---
print()
print('=== DECAIMENTO DA AMPLITUDE ===')
lv = np.log(picos_a)
A1 = np.polyfit(picos_t, lv, 1)        # viscoso: ln(A) linear no tempo
A2 = np.polyfit(picos_t, picos_a, 1)   # coulomb: A linear no tempo
r_visc = np.corrcoef(picos_t, lv)[0,1]**2
r_coul = np.corrcoef(picos_t, picos_a)[0,1]**2
print(f'ajuste exponencial (viscoso): R2={r_visc:.4f}  taxa={A1[0]:.5f} 1/s')
print(f'ajuste linear     (coulomb): R2={r_coul:.4f}  taxa={A2[0]:.5f} rad/s')
print(f'  -> b_viscoso equivalente = {-2*A1[0]:.4f} 1/s')
np.save('analise/picos.npy', np.c_[picos_t, picos_a])
np.save('analise/periodos.npy', np.c_[tper[ok], per_ok, amp_ok])
print(f'\nT0={T0:.5f} w0={w0:.5f} L={L:.5f} b_est={-2*A1[0]:.5f}')
