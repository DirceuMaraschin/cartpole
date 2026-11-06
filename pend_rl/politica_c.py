#!/usr/bin/env python3
"""
Carrega politica.c via ctypes: o simulador passa a rodar o C de produção.

POR QUE ISTO EXISTE. Até agora a validação em malha fechada rodava
`PoliticaNumpy` (numpy, float64) e a placa vai rodar `politica.c` (C, float32).
Duas implementações da mesma rede, validadas uma contra a outra só em vetores
estáticos. Foi exatamente esse arranjo que fez `teste_travada.py` passar em
falso: o teste remendava `AtorCritico.agir`, que já não estava no laço.

Aqui o `avaliar_sw.py` chama o MESMO arquivo .c que o compilador do ESP32 vai
compilar. Se o C estiver errado, `frac_pe` cai -- não há como o teste passar
vazio.

O QUE ISTO NÃO PROVA. Latência. O x86 deste container não diz nada sobre o
Xtensa; para isso existe o sketch de benchmark. Aqui se mede só fidelidade
numérica e comportamento em malha fechada.

Uso:
    python politica_c.py                    # compila as duas variantes e afere
    python politica_c.py --lut              # só a variante com LUT

Como módulo:
    from politica_c import PoliticaC
    pol = PoliticaC(lut=False)
    acao = pol.agir(obs)                    # mesma assinatura de PoliticaNumpy
"""

import argparse
import ctypes
import os
import subprocess
import sys
import time

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
# A fonte dos pesos e a pasta GRAVADA: uma copia so, sem divergencia
# possivel entre o que roda na placa e o que o simulador valida.
DIR_C = os.path.join(AQUI, 'firmware', 'pendulo_rl')
# O .so vai para a pasta de teste, nao para a do sketch.
DIR_BUILD = os.path.join(AQUI, 'firmware', 'politica')
DIM_OBS = 6

_pf = ctypes.POINTER(ctypes.c_float)


class ErroDeCompilacao(RuntimeError):
    pass


class PoliticaC:
    """O ator em C, carregado como biblioteca compartilhada.

    lut=False  -> tanhf() da libm, o padrão
    lut=True   -> -DPOLITICA_TANH_LUT, a tabela de 256 intervalos
    """

    def __init__(self, dir_c=DIR_C, lut=False, cc='cc', forcar=False,
                 flags=('-O2', '-Wall', '-Wextra')):
        self.dir_c = dir_c
        self.lut = bool(lut)
        self.fonte = os.path.join(dir_c, 'politica.c')
        if not os.path.exists(self.fonte):
            raise FileNotFoundError(
                f'nao achei {self.fonte}\n'
                f'rode primeiro:  python exportar_c.py')

        self.so = self._compilar(cc, flags, forcar)
        self.lib = ctypes.CDLL(self.so)

        self.lib.politica_iniciar.restype = None
        self.lib.politica_iniciar.argtypes = []
        self.lib.politica_bruta.restype = ctypes.c_float
        self.lib.politica_bruta.argtypes = [_pf]
        self.lib.politica_aceleracao.restype = ctypes.c_float
        self.lib.politica_aceleracao.argtypes = [_pf]

        # Chamar explicitamente, mesmo havendo auto-init no C: assim a cópia
        # para "SRAM" acontece fora de qualquer medição de tempo.
        self.lib.politica_iniciar()

    # ------------------------------------------------------------------
    def _compilar(self, cc, flags, forcar):
        nome = 'libpolitica_lut.so' if self.lut else 'libpolitica.so'
        os.makedirs(DIR_BUILD, exist_ok=True)
        destino = os.path.join(DIR_BUILD, nome)

        if not forcar and os.path.exists(destino):
            if os.path.getmtime(destino) > os.path.getmtime(self.fonte):
                return destino       # já está em dia

        cmd = [cc, *flags, '-fPIC', '-shared', '-o', destino, self.fonte, '-lm']
        if self.lut:
            cmd.insert(1, '-DPOLITICA_TANH_LUT')
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            raise ErroDeCompilacao(
                f'falhou: {" ".join(cmd)}\n{r.stderr}')
        if r.stderr.strip():
            print(f'  aviso do compilador:\n{r.stderr.strip()}', file=sys.stderr)
        return destino

    # ------------------------------------------------------------------
    def agir(self, obs):
        """Saída crua, em unidades normalizadas.

        Devolve array (1,) float32 para ser intercambiável com
        AtorCritico.agir(deterministico=True)[0] e com PoliticaNumpy.agir.
        """
        o = np.ascontiguousarray(obs, dtype=np.float32).reshape(-1)
        if o.size != DIM_OBS:
            raise ValueError(f'esperava {DIM_OBS} entradas, recebi {o.size}')
        y = self.lib.politica_bruta(o.ctypes.data_as(_pf))
        return np.array([y], dtype=np.float32)

    def aceleracao(self, obs):
        """m/s2, já com clip em +-1 e escala por A_MAX -- como o firmware."""
        o = np.ascontiguousarray(obs, dtype=np.float32).reshape(-1)
        return float(self.lib.politica_aceleracao(o.ctypes.data_as(_pf)))

    def __repr__(self):
        return f'<PoliticaC lut={self.lut} so={os.path.basename(self.so)}>'


# ======================================================================
# aferição contra o torch
# ======================================================================
def referencia_torch(caminho):
    import torch
    from ppo import AtorCritico
    rede = AtorCritico(DIM_OBS, 1, oculta=64)
    rede.load_state_dict(torch.load(caminho, map_location='cpu',
                                    weights_only=True)['rede'])
    rede.eval()

    def f(obs):
        with torch.no_grad():
            t = torch.as_tensor(np.asarray(obs, dtype=np.float32))
            return rede.ator(t).numpy()
    return f


def amostrar(n, semente=777):
    """Vetores no envelope real: sin e cos vêm de um psi sorteado."""
    rng = np.random.default_rng(semente)
    psi = rng.uniform(-np.pi, np.pi, n)
    return np.stack([rng.uniform(-1.1, 1.1, n),
                     rng.uniform(-1.1, 1.1, n),
                     np.sin(psi), np.cos(psi),
                     rng.uniform(-1.1, 1.1, n),
                     rng.uniform(-1.0, 1.0, n)], axis=1).astype(np.float32)


def aferir(pol, ref, obs, nome):
    y_c = np.array([pol.agir(o)[0] for o in obs], dtype=np.float64)
    y_t = ref(obs).ravel().astype(np.float64)

    e = np.abs(y_c - y_t)
    # o erro que importa: depois do clip e da escala
    a_c = np.clip(y_c, -1, 1) * 8.0
    a_t = np.clip(y_t, -1, 1) * 8.0
    ea = np.abs(a_c - a_t)

    t0 = time.perf_counter()
    for o in obs[:2000]:
        pol.agir(o)
    dt = (time.perf_counter() - t0) / 2000 * 1e6

    print(f'{nome:<22} erro cru max {e.max():.3e}  medio {e.mean():.3e}')
    print(f'{"":<22} erro m/s2 max {ea.max():.3e} de 8,0  '
          f'({100*ea.max()/8:.3f}%)')
    print(f'{"":<22} {dt:.1f} us/chamada (inclui o custo do ctypes)')
    return e.max(), ea.max()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--modelo', default='modelos_sw/melhor.pt')
    ap.add_argument('--dir-c', default=DIR_C)
    ap.add_argument('--n', type=int, default=4000)
    ap.add_argument('--lut', action='store_true', help='só a variante com LUT')
    ap.add_argument('--forcar', action='store_true', help='recompilar sempre')
    args = ap.parse_args()

    sys.path.insert(0, AQUI)
    obs = amostrar(args.n)
    ref = referencia_torch(args.modelo)

    print(f'Modelo   {args.modelo}')
    print(f'Fonte C  {os.path.join(args.dir_c, "politica.c")}')
    print(f'Vetores  {args.n} no envelope (sin^2+cos^2 = 1)\n')

    variantes = [(True, 'C + LUT de tanh')] if args.lut else \
                [(False, 'C + tanhf (libm)'), (True, 'C + LUT de tanh')]

    pior = {}
    for lut, nome in variantes:
        pol = PoliticaC(dir_c=args.dir_c, lut=lut, forcar=args.forcar)
        pior[nome] = aferir(pol, ref, obs, nome)
        print()

    ruim = [n for n, (e, _) in pior.items()
            if ('LUT' not in n and e > 1e-5) or ('LUT' in n and e > 1e-2)]
    if ruim:
        print(f'FALHOU: {", ".join(ruim)}')
        print('Erro acima do arredondamento: procure transposicao de W ou')
        print('troca de entradas na observacao, nao precisao.')
        return 1

    print('OK. Proximo: comparar_c.py, que roda isto em malha fechada.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
