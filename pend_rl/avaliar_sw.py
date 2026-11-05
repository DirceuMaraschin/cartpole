#!/usr/bin/env python3
"""
Valida o agente COMPLETO (swing-up + equilíbrio) antes da bancada.

Seis baterias, cada uma respondendo a uma pergunta que a bancada vai fazer:

  1. Sobe e fica?            -- o básico, com randomização de domínio ligada
  2. Quanto do trilho usa?   -- SEGURANÇA: se ele precisa de 240 mm para subir,
                                a bancada vai bater nas pontas
  3. Tremula?                -- foi o que o operador reclamou do agente v1
  4. Recupera de empurrão?   -- a etapa 8 da bancada
  5. Aguenta mundo errado?   -- L_eq, atrito e escala fora do nominal
  6. Sobe de onde parar?     -- da haste pendurada em qualquer posição do carro

Uso:
    python avaliar_sw.py                       # modelos_sw/melhor.pt
    python avaliar_sw.py modelos_sw/agente.pt
"""
import sys

import numpy as np
import torch

sys.path.insert(0, '.')
torch.set_num_threads(1)

from pendulo_env_sw import PenduloSwingUp, PSID_ESC
from ppo import AtorCritico
import pendulo_env as E

PE = np.radians(10.0)


def carregar(caminho):
    r = AtorCritico(6, 1)
    r.load_state_dict(torch.load(caminho, map_location='cpu',
                                 weights_only=True)['rede'])
    r.eval()
    return lambda o: r.agir(o, deterministico=True)[0]


def rodar(pol, env, obs=None, passos=None, empurrao=None, pendurado=True):
    obs = env.reset(pendurado=pendurado) if obs is None else obs
    n = passos or env.max_passos
    ps, xs, aa, alt = [], [], [], []
    prim, a_ant, caiu = None, 0.0, False
    for k in range(n):
        if empurrao and k == empurrao[0]:
            env.omega += empurrao[1]
        obs, r, caiu, corte, info = env.step(pol(obs))
        ps.append(abs(info['psi'])); xs.append(info['x'])
        aa.append(info['a']); alt.append(info['altura'])
        if abs(info['psi']) < PE and prim is None:
            prim = k
        if caiu or corte:
            break
    aa = np.array(aa)
    epe = np.array(ps) < PE
    meio = len(ps) // 2
    return dict(
        passos=k + 1, caiu=caiu, subiu=prim is not None,
        t_subida=prim * 0.020 if prim is not None else np.nan,
        frac_pe=float(epe[meio:].mean()) if len(epe) > meio else 0.0,
        psi_pe=np.degrees(np.mean([p for p, e in zip(ps, epe) if e])) if epe.any() else np.nan,
        x_max=float(np.max(np.abs(xs))) * 1000,
        x_med=float(np.mean(np.abs(xs))) * 1000,
        suav=float(np.abs(np.diff(aa)).mean()) if len(aa) > 2 else np.nan,
        inv_s=float(np.sum(np.diff(np.sign(aa)) != 0) / ((k + 1) * 0.020)),
        a_max=float(np.abs(aa).max()),
    )


def bateria(pol, n=30, semente=4242, **kw):
    env = PenduloSwingUp(max_passos=1200, semente=semente, **kw)
    return [rodar(pol, env) for _ in range(n)]


def media(rs, k):
    v = [r[k] for r in rs if not (isinstance(r[k], float) and np.isnan(r[k]))]
    return float(np.mean(v)) if v else float('nan')


def main():
    cam = sys.argv[1] if len(sys.argv) > 1 else 'modelos_sw/melhor.pt'
    pol = carregar(cam)
    print(f'=== VALIDACAO DO AGENTE COMPLETO: {cam} ===\n')

    print('--- 1. SOBE E FICA? (30 episodios de 24 s, dominio aleatorio) ---')
    rs = bateria(pol, n=30, perturbacoes=False, frac_topo=0.0)
    print(f'  sobe em            {100*np.mean([r["subiu"] for r in rs]):5.1f}% dos episodios')
    print(f'  tempo de subida    {media(rs,"t_subida"):5.2f} s   '
          f'(controlador de energia na bancada: 3,55 s)')
    print(f'  fica em pe         {100*media(rs,"frac_pe"):5.1f}% da segunda metade')
    print(f'  <|psi|> em pe      {media(rs,"psi_pe"):5.2f} graus  '
          f'(agente v1 na bancada: 1,46)')
    print(f'  bateu no trilho em {100*np.mean([r["caiu"] for r in rs]):5.1f}% dos episodios')

    print('\n--- 2. QUANTO DO TRILHO USA? (seguranca da bancada) ---')
    xm = np.array([r['x_max'] for r in rs])
    print(f'  |x| maximo: media {xm.mean():.0f} mm | pior {xm.max():.0f} mm '
          f'| limite 240 mm')
    print(f'  episodios passando de 200 mm: {100*np.mean(xm>200):.0f}%')
    print(f'  <|x|> em regime: {media(rs,"x_med"):.0f} mm')
    if xm.max() > 230:
        print('  >> ATENCAO: encosta no limite. Na bancada o firmware vai parar.')
    else:
        print(f'  >> Folga de {240-xm.max():.0f} mm no pior caso.')

    print('\n--- 3. TREMULA? (a reclamacao do operador sobre o v1) ---')
    print(f'  <|da|>        {media(rs,"suav"):5.2f} m/s2 por passo   '
          f'(v1: 1,72 no simulador, 2,31 na bancada)')
    print(f'  inversoes/s   {media(rs,"inv_s"):5.1f}                 '
          f'(v1: 17,3 no simulador, 20,7 na bancada)')
    print(f'  |a| maximo    {media(rs,"a_max"):5.2f} de 8,0')

    print('\n--- 4. RECUPERA DE EMPURRAO? (etapa 8) ---')
    print(f'{"empurrao":>12} {"recupera":>10} {"psi pico":>10}')
    for dom in (0.5, 1.0, 1.5, 2.0, 3.0):
        env = PenduloSwingUp(max_passos=1200, semente=777, perturbacoes=False,
                             frac_topo=1.0)
        ok, pk = 0, []
        for _ in range(20):
            # sobe primeiro, depois chuta e ve se volta
            r = rodar(pol, env, pendurado=False, empurrao=(400, dom))
            ok += (r['frac_pe'] > 0.8) and not r['caiu']
            pk.append(r['psi_pe'])
        print(f'{dom:10.1f} rad/s {100*ok/20:9.0f}% {np.nanmean(pk):9.2f}d')

    print('\n--- 5. AGUENTA MUNDO ERRADO? ---')
    print(f'{"mundo":<34} {"sobe":>7} {"em pe":>7} {"psi":>7}')
    casos = [
        ('nominal (sem randomizacao)', dict(dominio_aleatorio=False)),
        ('randomizado (como treinou)', dict(dominio_aleatorio=True)),
    ]
    for nome, kw in casos:
        r2 = bateria(pol, n=20, perturbacoes=False, frac_topo=0.0, **kw)
        print(f'{nome:<34} {100*np.mean([r["subiu"] for r in r2]):6.0f}% '
              f'{100*media(r2,"frac_pe"):6.0f}% {media(r2,"psi_pe"):6.2f}d')

    print('\n--- 6. SOBE DE ONDE O CARRINHO PARAR? ---')
    print(f'{"x inicial":>11} {"sobe":>7} {"t subida":>9} {"x max":>8}')
    for x0 in (-0.20, -0.10, 0.0, 0.10, 0.20):
        env = PenduloSwingUp(max_passos=1200, semente=321, perturbacoes=False)
        ok, ts, xx = 0, [], []
        for _ in range(10):
            env.reset(psi0=np.pi, x0=x0)
            r = rodar(pol, env, obs=env._observar())
            ok += r['subiu']; xx.append(r['x_max'])
            if r['subiu']:
                ts.append(r['t_subida'])
        print(f'{x0*1000:9.0f}mm {100*ok/10:6.0f}% '
              f'{np.mean(ts) if ts else float("nan"):8.2f}s {np.mean(xx):6.0f}mm')


if __name__ == '__main__':
    main()
