# Scripts de análise

Rode da raiz do projeto, com os CSVs em `logs/`.

| Script | O que faz |
|---|---|
| `ident_livre.py`   | Período por cruzamento de zero, extrapolação elíptica → L_eq; tipo de atrito |
| `ajuste_atrito.py` | Tentativa (falha didática) de ajustar tudo por RMS global — mostra por que não funciona |
| `ajuste2.py`       | Ajuste correto: b e c pelo envelope de decaimento |
| `valida3.py`       | Validação em janelas de 5 s da oscilação livre |
| `ident_pid.py`     | Conferência do PID, quantização do encoder, ruído de ω, atraso |
| `valida_swing.py`  | Validação com `a_ms2` (errada — mostra o efeito da saturação) |
| `valida_swing2.py` | Validação correta com `dv/dt` |
| `figura.py`        | Gera `validacao_modelo.png` |
| `teste_pid.py`     | Portão de qualidade: PID do firmware dentro do simulador |
| `teste_taxa.py`    | Efeito da taxa de controle (1 kHz → 25 Hz) |
| `teto_50hz.py`     | Busca dos melhores ganhos para 50 Hz |

Dependências: `numpy`, `pandas`, `scipy`, `matplotlib`.
