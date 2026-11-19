function [kan, hist] = ident_treinar_kan(D, cfg)
%IDENT_TREINAR_KAN  Treina a KAN que prevê a planta um passo à frente (sem pykan).
%   [kan, hist] = ident_treinar_kan(D, cfg)      D = ident_gerar_dados(...)
%
%   Modelo (série-paralelo / NARX de 1 passo):
%       x(k+1) = x(k) + KAN( dx(k), phi(k), dth(k), u(k) )     (4 incrementos)
%   A KAN tem o mesmo formato do pykan (SiLU de base + B-splines por aresta,
%   scale_base, scale_sp, afim de nó), então roda com kan_eval.h no ESP32 e com
%   kan_forward/ident_kan_forward no MATLAB. sp e a ficam fixos em 1; treinam-se
%   coef, sb e c com Adam (gradiente calculado à mão) e o grid da 2a camada é
%   reajustado à faixa das ativações ocultas durante o treino, como o pykan faz.
%
%   Campos de cfg (opcionais): largura ([4 8 4]), G (8), k (3), iter (3000),
%   lr (0.01), lr_fim (5e-4), lamb (1e-6), frac_val (0.15), iter_grid
%   ([100 300 800 1600]), semente (0), mostrar (250), max_amostras (20000:
%   subamostra aleatória do treino, para o treino não ficar lento),
%   th_eq (pi: ângulo de equilíbrio usado no regressor phi = wrap(th - th_eq)),
%   ep_val ([]: episódios de validação; vazio = os últimos frac_val),
%   norm ('max': entrada/max|entrada| | 'std': entrada/(n_std*desvio)), n_std (3),
%   linear (false): true = KAN residual, dS = [v phi w u 1]*lin + KAN(...), com lin
%   ajustado antes por mínimos quadrados (a KAN aprende só o que o linear não explica).
%   Só 2 camadas são suportadas aqui (largura = [4 H 4]).

if nargin < 2, cfg = struct(); end
pad = struct('largura', [4 8 4], 'G', 8, 'k', 3, 'iter', 3000, 'lr', 0.01, ...
    'lr_fim', 5e-4, 'lamb', 1e-6, 'frac_val', 0.15, 'iter_grid', [100 300 800 1600], ...
    'semente', 0, 'mostrar', 250, 'max_amostras', 20000, 'th_eq', pi, 'ep_val', [], ...
    'norm', 'max', 'n_std', 3, 'linear', false, 'sincos', false);
nomes = fieldnames(pad);
for i = 1:numel(nomes)
    if ~isfield(cfg, nomes{i}), cfg.(nomes{i}) = pad.(nomes{i}); end
end
ne_esp = 4 + cfg.sincos;
if numel(cfg.largura) ~= 3 || cfg.largura(1) ~= ne_esp || cfg.largura(3) ~= 4
    error('largura deve ser [%d H 4] (sincos = %d).', ne_esp, cfg.sincos);
end
rng_ant = rng; rng(cfg.semente);
k = cfg.k; G = cfg.G; nh = cfg.largura(2); no = 4; ne = cfg.largura(1); nb = G + k;

% ---------------- normalização ----------------
kan.th_eq = cfg.th_eq;
kan.sincos = logical(cfg.sincos);
kan.Ts = D.cfg.Ts;
kan.ent_ganho = ones(1, ne); kan.ent_desloc = zeros(1, ne);
Fbruto = ident_regressores(kan, D.S, D.U);          % com ganho 1: [dx phi dth u]
if strcmpi(cfg.norm, 'std')
    % robusta: n_std desvios-padrão -> +-1 (dados concentrados perto de 0 ganham
    % resolução de grid; valores maiores caem na extensão do grid / só no SiLU)
    kan.ent_ganho = 1./max(cfg.n_std*std(Fbruto, 0, 1), 1e-6);
else
    kan.ent_ganho = 1./max(max(abs(Fbruto), [], 1), 1e-6);
end
X = ident_regressores(kan, D.S, D.U);               % ~[-1, 1]
T = D.S1 - D.S;                                     % incrementos [dx_ dv dth_ dw]

% treino/validação separados por episódio
eps_ = unique(D.ep);
if isempty(cfg.ep_val)
    n_val = max(1, round(cfg.frac_val*numel(eps_)));
    ep_val = eps_(end-n_val+1:end);
else
    ep_val = cfg.ep_val(:); n_val = numel(ep_val);
end
iv = ismember(D.ep, ep_val); it_ = ~iv;

% parte linear (KAN residual): dS = [F 1]*lin + KAN(xn), F = regressores sem normalizar
kan.lin = zeros(ne + 1, 4);
if cfg.linear
    Fl = [Fbruto, ones(size(Fbruto, 1), 1)];
    kan.lin = (Fl(it_, :).'*Fl(it_, :) + 1e-9*eye(ne + 1)) \ (Fl(it_, :).'*T(it_, :));
    T = T - Fl*kan.lin;                              % a KAN aprende só o resíduo
    fprintf('Parte linear ajustada (mínimos quadrados); a KAN aprende o resíduo.\n');
end
kan.sai_desloc = mean(T, 1);
kan.sai_ganho  = max(std(T, 0, 1), 1e-9);
Tn = bsxfun(@rdivide, bsxfun(@minus, T, kan.sai_desloc), kan.sai_ganho);
Xtr = X(it_, :); Ttr = Tn(it_, :); Xva = X(iv, :); Tva = Tn(iv, :);
if size(Xtr, 1) > cfg.max_amostras            % subamostra: treino mais rápido
    sel = randperm(size(Xtr, 1), cfg.max_amostras);
    Xtr = Xtr(sel, :); Ttr = Ttr(sel, :);
end
if size(Xva, 1) > cfg.max_amostras/2
    sel = randperm(size(Xva, 1), round(cfg.max_amostras/2));
    Xva = Xva(sel, :); Tva = Tva(sel, :);
end
N = size(Xtr, 1);
fprintf('Treino: %d amostras | validação: %d amostras (%d episódios)\n', N, size(Xva, 1), n_val);

% ---------------- inicialização ----------------
gl = grid_unif(-1, 1, G, k);
P.coef1 = 0.1*randn(ne, nh, nb);
P.sb1   = (2*rand(ne, nh) - 1)/sqrt(ne);
P.c1    = zeros(1, nh);
P.coef2 = 0.1*randn(nh, no, nb);
P.sb2   = (2*rand(nh, no) - 1)/sqrt(nh);
P.c2    = zeros(1, no);
grid1 = repmat(gl, ne, 1);
grid2 = repmat(gl, nh, 1);

% camada 1: entradas fixas -> bases pré-calculadas
B1 = cell(1, ne); silu1 = zeros(N, ne);
for i = 1:ne
    B1{i} = ident_kan_bsplines(Xtr(:, i), grid1(i, :), k);
    silu1(:, i) = Xtr(:, i)./(1 + exp(-Xtr(:, i)));
end

campos = fieldnames(P);
for f = 1:numel(campos)
    Mo.(campos{f}) = zeros(size(P.(campos{f})));
    Ve.(campos{f}) = zeros(size(P.(campos{f})));
end
b1a = 0.9; b2a = 0.999; epsA = 1e-8;
hist.perda = nan(cfg.iter, 1); hist.val = nan(cfg.iter, 1);
melhor = inf; Pmelhor = P; grid2melhor = grid2;
t0 = tic;
for it = 1:cfg.iter
    % ---- atualização do grid da camada 2 (preserva a função das splines) ----
    if any(it == cfg.iter_grid)
        H = camada1(P, B1, silu1, N, nh);
        for i = 1:nh
            lo = min(H(:, i)); hi = max(H(:, i));
            marg = 0.1*(hi - lo) + 1e-3;
            gn = grid_unif(lo - marg, hi + marg, G, k);
            Bv = ident_kan_bsplines(H(:, i), grid2(i, :), k);
            Bn = ident_kan_bsplines(H(:, i), gn, k);
            Ci = reshape(P.coef2(i, :, :), no, nb);
            alvo = Bv*Ci.';
            Cn = (Bn.'*Bn + 1e-6*eye(nb)) \ (Bn.'*alvo);  % nb x no
            P.coef2(i, :, :) = reshape(Cn.', [1 no nb]);
            grid2(i, :) = gn;
        end
        Mo.coef2(:) = 0; Ve.coef2(:) = 0;
    end

    % ---- forward ----
    H = camada1(P, B1, silu1, N, nh);
    S2 = zeros(N, no);
    B2 = cell(1, nh); dB2 = cell(1, nh);
    sg = 1./(1 + exp(-H));
    silu2 = H.*sg; dsilu2 = sg.*(1 + H.*(1 - sg));
    for i = 1:nh
        [B2{i}, dB2{i}] = ident_kan_bsplines(H(:, i), grid2(i, :), k);
        Ci = reshape(P.coef2(i, :, :), no, nb);
        S2 = S2 + silu2(:, i)*P.sb2(i, :) + B2{i}*Ci.';
    end
    Y = bsxfun(@plus, S2, P.c2);
    E = Y - Ttr;
    perda = mean(E(:).^2);
    hist.perda(it) = perda;

    % ---- backward ----
    dY = 2*E/numel(E);
    g.c2 = sum(dY, 1);
    g.coef2 = zeros(size(P.coef2)); g.sb2 = zeros(size(P.sb2));
    dH = zeros(N, nh);
    for i = 1:nh
        Ci = reshape(P.coef2(i, :, :), no, nb);
        g.coef2(i, :, :) = reshape((B2{i}.'*dY).', [1 no nb]);
        g.sb2(i, :) = silu2(:, i).'*dY;
        dH(:, i) = (dY*P.sb2(i, :).').*dsilu2(:, i) + sum((dY*Ci).*dB2{i}, 2);
    end
    g.c1 = sum(dH, 1);
    g.coef1 = zeros(size(P.coef1)); g.sb1 = zeros(size(P.sb1));
    for i = 1:ne
        g.coef1(i, :, :) = reshape((B1{i}.'*dH).', [1 nh nb]);
        g.sb1(i, :) = silu1(:, i).'*dH;
    end
    g.coef1 = g.coef1 + 2*cfg.lamb*P.coef1;
    g.coef2 = g.coef2 + 2*cfg.lamb*P.coef2;

    % ---- Adam com taxa em cosseno ----
    lr = cfg.lr_fim + 0.5*(cfg.lr - cfg.lr_fim)*(1 + cos(pi*(it - 1)/cfg.iter));
    for f = 1:numel(campos)
        c = campos{f};
        Mo.(c) = b1a*Mo.(c) + (1 - b1a)*g.(c);
        Ve.(c) = b2a*Ve.(c) + (1 - b2a)*g.(c).^2;
        mh = Mo.(c)/(1 - b1a^it); vh = Ve.(c)/(1 - b2a^it);
        P.(c) = P.(c) - lr*mh./(sqrt(vh) + epsA);
    end

    % ---- validação ----
    if mod(it, 25) == 0 || it == cfg.iter
        kv = montar(kan, P, grid1, grid2, k);
        Ev = ident_kan_forward(kv, Xva) - Tva;
        hist.val(it) = mean(Ev(:).^2);
        if hist.val(it) < melhor && it > max(cfg.iter_grid)
            melhor = hist.val(it); Pmelhor = P; grid2melhor = grid2;
        end
    end
    if mod(it, cfg.mostrar) == 0 || it == 1
        fprintf('  iter %5d  perda treino %.3e  validação %.3e  lr %.1e  (%.0f s)\n', ...
            it, perda, hist.val(max(1, it - mod(it, 25))), lr, toc(t0));
    end
end
if isinf(melhor), Pmelhor = P; grid2melhor = grid2; end
kan = montar(kan, Pmelhor, grid1, grid2melhor, k);
kan.largura = cfg.largura; kan.G = G;
kan.regressores = {'dx', 'phi = wrap(th - pi)', 'dth', 'u'};
kan.saidas = {'x(k+1)-x(k)', 'dx(k+1)-dx(k)', 'th(k+1)-th(k)', 'dth(k+1)-dth(k)'};
hist.melhor_val = melhor;
rng(rng_ant);
fprintf('Treino concluído em %.0f s. Melhor perda de validação (normalizada): %.3e\n', toc(t0), melhor);
end

% =====================================================================
function H = camada1(P, B1, silu1, N, nh)
S = zeros(N, nh);
nb = size(P.coef1, 3);
for i = 1:numel(B1)
    Ci = reshape(P.coef1(i, :, :), nh, nb);
    S = S + silu1(:, i)*P.sb1(i, :) + B1{i}*Ci.';
end
H = bsxfun(@plus, S, P.c1);
end

function kan = montar(kan, P, grid1, grid2, k)
kan.k = k;
kan.camadas = cell(1, 2);
kan.camadas{1} = struct('grid', grid1, 'coef', P.coef1, 'sb', P.sb1, ...
    'sp', ones(size(P.sb1)), 'a', ones(1, size(P.sb1, 2)), 'c', P.c1);
kan.camadas{2} = struct('grid', grid2, 'coef', P.coef2, 'sb', P.sb2, ...
    'sp', ones(size(P.sb2)), 'a', ones(1, size(P.sb2, 2)), 'c', P.c2);
end

function g = grid_unif(lo, hi, G, k)
% grid estendido do pykan: G intervalos em [lo, hi] + k pontos de cada lado
h = (hi - lo)/G;
g = lo + (-k:G+k)*h;
end
