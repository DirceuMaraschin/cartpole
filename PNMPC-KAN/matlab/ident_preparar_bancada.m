function [kan, V] = ident_preparar_bancada(arquivo, varargin)
%IDENT_PREPARAR_BANCADA  Treina a KAN da planta com um ensaio da BANCADA e exporta p/ o ESP32.
%
%   kan = ident_preparar_bancada                     % ensaio_20261119_154800.csv
%   kan = ident_preparar_bancada('..\Planta_smiluação\outro.csv', 'iter', 4000)
%
%   Planta da bancada (carrinho com motor de passo, firmware pid_IA):
%     estado  s = [x (m), v (m/s), theta (rad, 0 = pendurado), omega (rad/s)]
%     entrada u = a_cmd (aceleração comandada do carrinho, m/s^2)
%   Modelo: s(k+1) = s(k) + [v th w a 1]*lin + KAN(v, wrap(theta), omega, a_cmd)  (KAN residual 4-H-4)
%
%   1) lê o ensaio (ident_ler_bancada)
%   2) treina deixando os segmentos 'seg_val' de fora (validação)
%   3) avalia no ensaio inteiro: 1 passo à frente e simulação livre
%   4) salva kan_bancada.mat e gera esp32/kan_planta_esp32/kan_planta_pesos.h
%
%   Opções: H (6), G (5), iter (3000), seg_val ([6 9]), exportar (true), graficos (true),
%           linear (true: KAN residual sobre um modelo linear), norm ('max'), lamb (1e-4)

aqui = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(arquivo)
    arquivo = fullfile(aqui, '..', 'Planta_smiluação', 'ensaio_20261119_154800.csv');
end
op = struct('H', 6, 'G', 5, 'iter', 3000, 'seg_val', [6 9], 'exportar', true, 'graficos', true, ...
    'linear', true, 'norm', 'max', 'lamb', 1e-4);
for i = 1:2:numel(varargin)
    if ~isfield(op, varargin{i}), error('Opção desconhecida: %s', varargin{i}); end
    op.(varargin{i}) = varargin{i+1};
end

% ---------- 1) dados ----------
D = ident_ler_bancada(arquivo);

% ---------- 2) treino ----------
[kan, hist] = ident_treinar_kan(D, struct('largura', [4 op.H 4], 'G', op.G, ...
    'iter', op.iter, 'th_eq', 0, 'ep_val', op.seg_val, 'linear', op.linear, ...
    'norm', op.norm, 'lamb', op.lamb));
kan.nome = 'bancada';
kan.arquivo_dados = arquivo;
kan.regressores = {'v', 'phi = wrap(theta)', 'omega', 'a_cmd'};
kan.saidas = {'x(k+1)-x(k)', 'v(k+1)-v(k)', 'theta(k+1)-theta(k)', 'omega(k+1)-omega(k)'};
kan.rotulo_u = 'a_{cmd} [m/s^2]';

% ---------- 3) avaliação no ensaio inteiro ----------
V = ident_replay(kan, D.serie);
esc = [1000 1000 180/pi 180/pi];
fprintf('\nEnsaio inteiro (%d amostras):\n', numel(V.t));
fprintf('                 %10s %10s %10s %10s\n', 'x [mm]', 'v [mm/s]', 'th [graus]', 'w [g/s]');
fprintf('  RMSE 1 passo   %10.4f %10.4f %10.4f %10.4f\n', V.rmse_1p.*esc);
fprintf('  RMSE livre     %10.4f %10.4f %10.4f %10.4f\n', V.rmse_livre.*esc);
iv = ismember(V.seg(2:end), op.seg_val);
e1 = V.P1(2:end, :) - V.S(2:end, :);
fprintf('  RMSE 1 passo só nos segmentos de validação %s:\n', mat2str(op.seg_val));
fprintf('                 %10.4f %10.4f %10.4f %10.4f\n', sqrt(mean(e1(iv, :).^2, 1)).*esc);
fprintf('  (quantização do encoder: 0,15 graus)\n');

save(fullfile(aqui, 'kan_bancada.mat'), 'kan', 'hist', 'V');
fprintf('KAN salva em %s\n', fullfile(aqui, 'kan_bancada.mat'));

% ---------- 4) exportação ----------
if op.exportar
    idx = round(linspace(2, numel(V.t) - 1, 16));
    ident_exportar_esp32(kan, V.S(idx, :), V.U(idx));
end

if op.graficos
    ident_plotar(V, 'Bancada: planta (ensaio) x KAN no MATLAB');
    ident_plotar_erro(V);
end
end

function V = ident_replay(kan, serie)
% previsões da KAN ao longo do ensaio gravado (mesma conta do modo 'dados' do HIL)
ok = serie.seg >= 0;
i0 = find(ok, 1); i1 = find(ok, 1, 'last');
r = i0:i1;
V.t = serie.t(r) - serie.t(i0); V.U = serie.U(r); V.S = serie.S(r, :); V.seg = serie.seg(r);
N = numel(r);
V.P1 = nan(N, 4); V.PL = nan(N, 4);
V.P1(1, :) = V.S(1, :); V.PL(1, :) = V.S(1, :);
V.P1(2:end, :) = ident_kan_prever(kan, V.S(1:end-1, :), V.U(1:end-1));
sl = V.S(1, :);
for k = 1:N-1
    sl = ident_kan_prever(kan, sl, V.U(k));
    V.PL(k+1, :) = sl;
end
V.rmse_1p    = sqrt(mean((V.P1(2:end, :) - V.S(2:end, :)).^2, 1));
V.rmse_livre = sqrt(mean((V.PL(2:end, :) - V.S(2:end, :)).^2, 1));
V.th_eq = kan.th_eq; V.rotulo_u = kan.rotulo_u;
V.nome = 'KAN no MATLAB (bancada)';
end
