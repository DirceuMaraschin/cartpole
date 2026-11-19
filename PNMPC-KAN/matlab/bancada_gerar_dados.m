function D = bancada_gerar_dados(cfg)
%BANCADA_GERAR_DADOS  Dados de treino da KAN com o pêndulo dando a VOLTA INTEIRA.
%   D = bancada_gerar_dados(cfg)
%
%   Usa a planta ajustada à bancada (bancada_param.mat: w0q, kl, c) e a MESMA medição
%   do firmware/bancada_planta (encoder de 0,15 grau e estimador de omega a cada 0,5 ms).
%   Simula cfg.n_ep episódios em paralelo (vetorizado), cada um começando num estado
%   aleatório (ângulo em toda a volta) e excitado por patamares aleatórios de a_cmd.
%   Saída no formato de ident_gerar_dados: D.S, D.U, D.S1 (medições), D.ep.
%
%   cfg (opcional): n_ep (500), dur_ep (2.5 s), Ts (0.01), A (6 m/s^2), dur ([0.02 0.3] s),
%   p_zero (0.2), w0_max (7 rad/s), v0_max (0.8 m/s), kv (2: a = nível - kv*v, mantém v
%   limitada), semente (0), medido (true: grava a medição do firmware; false: o estado
%   verdadeiro. Para o modelo do controlador use false: o omega medido depende do histórico
%   do estimador e o par (k, k+1) deixa de ser markoviano, o que distorce a KAN)

if nargin < 1, cfg = struct(); end
pad = struct('n_ep', 500, 'dur_ep', 2.5, 'Ts', 0.01, 'A', 6, 'dur', [0.02 0.3], ...
    'p_zero', 0.2, 'w0_max', 7, 'v0_max', 0.8, 'kv', 2, 'semente', 0, 'medido', true);
nomes = fieldnames(pad);
for i = 1:numel(nomes)
    if ~isfield(cfg, nomes{i}), cfg.(nomes{i}) = pad.(nomes{i}); end
end
aqui = fileparts(mfilename('fullpath'));
tmp = load(fullfile(aqui, 'bancada_param.mat'), 'p'); p = tmp.p;

rng_ant = rng; rng(cfg.semente);
E = cfg.n_ep; Nk = round(cfg.dur_ep/cfg.Ts); nsub = 20; h = cfg.Ts/nsub;
q = 2*pi/2400;

% estado verdadeiro (colunas = episódios)
x = zeros(1, E); v = cfg.v0_max*(2*rand(1, E) - 1);
th = pi*(2*rand(1, E) - 1); w = cfg.w0_max*(2*rand(1, E) - 1);
% patamares de a_cmd: nível e tempo restante de cada episódio
niv = sorteia_nivel(E, cfg); resta = sorteia_dur(E, cfg);

% medição (estimador do firmware)
thm = q*round(th/q); hist = repmat(thm, 8, 1); idx = 1; wm = zeros(1, E);
% pré-rolagem de 40 ms (a = 0) para o estimador de omega convergir
for i = 1:80
    [x, v, th, w] = rk4(x, v, th, w, zeros(1, E), h, p);
    [thm, wm, hist, idx] = medir(th, wm, hist, idx, q, h);
end

S = zeros(Nk, 4, E); U = zeros(Nk, E); S1 = zeros(Nk, 4, E);
for k = 1:Nk
    if cfg.medido, S(k, :, :) = reshape([x; v; thm; wm], 1, 4, E);
    else,          S(k, :, :) = reshape([x; v; th; w], 1, 4, E); end
    a = min(max(niv - cfg.kv*v, -cfg.A), cfg.A);
    U(k, :) = a;
    for i = 1:nsub
        [x, v, th, w] = rk4(x, v, th, w, a, h, p);
        [thm, wm, hist, idx] = medir(th, wm, hist, idx, q, h);
    end
    if cfg.medido, S1(k, :, :) = reshape([x; v; thm; wm], 1, 4, E);
    else,          S1(k, :, :) = reshape([x; v; th; w], 1, 4, E); end
    resta = resta - cfg.Ts;
    troca = resta <= 0;
    if any(troca)
        niv(troca) = sorteia_nivel(sum(troca), cfg);
        resta(troca) = sorteia_dur(sum(troca), cfg);
    end
end
rng(rng_ant);

D.S  = reshape(permute(S, [1 3 2]), [], 4);
D.S1 = reshape(permute(S1, [1 3 2]), [], 4);
D.U  = U(:);
D.ep = reshape(repmat(1:E, Nk, 1), [], 1);
D.cfg = cfg; D.cfg.p = p;
fprintf('Dados de volta inteira: %d episódios x %.1f s = %d amostras\n', E, cfg.dur_ep, numel(D.U));
fprintf('  |v| <= %.2f m/s, |omega| <= %.1f rad/s, |a| <= %.1f m/s^2, theta cobre %.0f graus\n', ...
    max(abs(D.S(:, 2))), max(abs(D.S(:, 4))), max(abs(D.U)), ...
    (max(mod(D.S(:, 3), 2*pi)) - min(mod(D.S(:, 3), 2*pi)))*180/pi);
end

function n = sorteia_nivel(m, cfg)
n = cfg.A*(2*rand(1, m) - 1);
n(rand(1, m) < cfg.p_zero) = 0;
end

function d = sorteia_dur(m, cfg)
d = cfg.dur(1) + (cfg.dur(2) - cfg.dur(1))*rand(1, m);
end

function [x, v, th, w] = rk4(x, v, th, w, a, h, p)
f = @(th, w) -p.w0q*sin(th) - p.kl*a.*cos(th) - p.c*w;
k1x = v;            k1v = a; k1t = w;            k1w = f(th, w);
k2x = v + h/2*k1v;  k2v = a; k2t = w + h/2*k1w;  k2w = f(th + h/2*k1t, w + h/2*k1w);
k3x = v + h/2*k2v;  k3v = a; k3t = w + h/2*k2w;  k3w = f(th + h/2*k2t, w + h/2*k2w);
k4x = v + h*k3v;    k4v = a; k4t = w + h*k3w;    k4w = f(th + h*k3t, w + h*k3w);
x  = x  + h/6*(k1x + 2*k2x + 2*k3x + k4x);
v  = v  + h/6*(k1v + 2*k2v + 2*k3v + k4v);
th = th + h/6*(k1t + 2*k2t + 2*k3t + k4t);
w  = w  + h/6*(k1w + 2*k2w + 2*k3w + k4w);
end

function [thm, wm, hist, idx] = medir(th, wm, hist, idx, q, h)
thm = q*round(th/q);
bruto = (thm - hist(idx, :))/(8*h);
wm = 0.5*wm + 0.5*bruto;
hist(idx, :) = thm;
idx = mod(idx, 8) + 1;
end
