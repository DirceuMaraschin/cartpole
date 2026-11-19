function [kan, V] = ident_preparar_kan(varargin)
%IDENT_PREPARAR_KAN  Gera os dados, treina a KAN da planta, valida e exporta para o ESP32.
%
%   kan = ident_preparar_kan                 % tudo com os valores padrão
%   kan = ident_preparar_kan('iter', 4000, 'H', 10)
%
%   1) ensaios em malha aberta com trem de pulsos ALEATÓRIO (pêndulo pendurado)
%   2) treino da KAN [4 H 4] que prevê o próximo estado (ident_treinar_kan)
%   3) validação com o trem de pulsos REGULAR do ensaio HIL (sinal não visto no treino):
%        - um passo à frente: a KAN recebe o estado real da planta a cada Ts
%        - simulação livre:  a KAN realimenta a própria previsão (sem ver a planta)
%   4) salva kan_planta.mat e gera esp32/kan_planta_esp32/kan_planta_pesos.h
%
%   Opções: Ts (0.01), n_ep (80), dur_ep (8), A_treino (2.5), H (8), G (8),
%           iter (3000), sinal (struct do ident_trem_pulsos, ensaio regular),
%           Tval (30), exportar (true), graficos (true)

op = struct('Ts', 0.01, 'n_ep', 80, 'dur_ep', 8, 'A_treino', 2.5, 'H', 8, 'G', 8, ...
    'iter', 3000, 'sinal', struct('tipo', 'regular', 'A', 2, 'largura', 0.1, ...
    'periodo', 2, 'atraso', 0.5), 'Tval', 30, 'exportar', true, 'graficos', true);
for i = 1:2:numel(varargin)
    if ~isfield(op, varargin{i}), error('Opção desconhecida: %s', varargin{i}); end
    op.(varargin{i}) = varargin{i+1};
end
aqui = fileparts(mfilename('fullpath'));

% ---------- 1) dados ----------
D = ident_gerar_dados(struct('Ts', op.Ts, 'n_ep', op.n_ep, 'dur_ep', op.dur_ep, 'A', op.A_treino));

% ---------- 2) treino ----------
[kan, hist] = ident_treinar_kan(D, struct('largura', [4 op.H 4], 'G', op.G, 'iter', op.iter));
kan.sinal_ensaio = op.sinal;

% ---------- 3) validação no ensaio HIL (planta x KAN) ----------
V = ident_validar(kan, op.sinal, op.Tval);
fprintf('\nValidação com trem de pulsos regular (A = %g N, %g s):\n', op.sinal.A, op.Tval);
fprintf('                 %10s %10s %10s %10s\n', 'x [mm]', 'dx [mm/s]', 'th [graus]', 'dth [g/s]');
esc = [1000 1000 180/pi 180/pi];
fprintf('  RMSE 1 passo   %10.4f %10.4f %10.4f %10.4f\n', V.rmse_1p.*esc);
fprintf('  RMSE livre     %10.4f %10.4f %10.4f %10.4f\n', V.rmse_livre.*esc);
fprintf('  faixa da planta: |x| <= %.3f m, |phi| <= %.1f graus\n', ...
    max(abs(V.S(:, 1))), rad2deg(max(abs(V.S(:, 3) - pi))));

save(fullfile(aqui, 'kan_planta.mat'), 'kan', 'hist', 'V');
fprintf('KAN salva em %s\n', fullfile(aqui, 'kan_planta.mat'));

% ---------- 4) exportação ----------
if op.exportar
    idx = round(linspace(1, size(V.S, 1) - 1, 16));
    ident_exportar_esp32(kan, V.S(idx, :), V.U(idx));
end

if op.graficos
    fig = ident_plotar(V, 'Validação offline (KAN no MATLAB)');
    try
        print(fig, fullfile(aqui, 'ident_validacao.png'), '-dpng', '-r110');
    catch
    end
    figure('Name', 'Treino da KAN', 'Color', 'w');
    semilogy(hist.perda, 'DisplayName', 'treino'); hold on;
    iv = ~isnan(hist.val); it = 1:numel(hist.val);
    semilogy(it(iv), hist.val(iv), 'o-', 'DisplayName', 'validação');
    grid on; xlabel('iteração'); ylabel('MSE normalizado'); legend show;
end
end

function V = ident_validar(kan, sinal, Tfinal)
p = parametros_planta();
Ts = kan.Ts; N = round(Tfinal/Ts);
V.t = (0:N)'*Ts;
V.U = ident_trem_pulsos(V.t, sinal);
V.S = zeros(N + 1, 4); V.P1 = nan(N + 1, 4); V.PL = nan(N + 1, 4);
s = [0; 0; pi; 0];
sl = s.';
V.S(1, :) = s.'; V.P1(1, :) = s.'; V.PL(1, :) = s.';
for k = 1:N
    V.P1(k + 1, :) = ident_kan_prever(kan, V.S(k, :), V.U(k));
    sl = ident_kan_prever(kan, sl, V.U(k));
    V.PL(k + 1, :) = sl;
    s = ident_planta_passo(s, V.U(k), Ts, p, 4);
    V.S(k + 1, :) = s.';
end
V.rmse_1p    = sqrt(mean((V.P1(2:end, :) - V.S(2:end, :)).^2, 1));
V.rmse_livre = sqrt(mean((V.PL(2:end, :) - V.S(2:end, :)).^2, 1));
end
