function dados = pnmpc_hil(varargin)
%PNMPC_HIL  HIL do PNMPC-KAN: planta da bancada no MATLAB, controlador no ESP32-S3.
%
%   Tarefa padrão: SWING-UP + EQUILÍBRIO. O pêndulo começa parado em 210 graus e a
%   referência de ângulo é 90 graus (convenção do SENSOR: 90 = em pé, 270 = pendurado).
%   O PNMPC-KAN usa pesos de energia longe do topo (swing-up) e de ângulo/posição perto
%   dele (equilíbrio), com penalidade suave no fim do trilho.
%
%   A planta (carrinho com motor de passo + pêndulo, ajustada ao ensaio 154800, com encoder
%   de 0,15 grau e o estimador de omega do firmware) roda aqui. A cada Ts o MATLAB manda a
%   medição e as referências; o ESP32 roda o PNMPC-KAN no núcleo 1 e devolve a_cmd.
%
%   Uso:
%     pnmpc_hil                                   % HIL na COM8, 210 -> 90 graus
%     pnmpc_hil('modo', 'sim')                    % mesmo PNMPC-KAN no MATLAB (lento)
%     pnmpc_hil('th0', 250, 'Tfinal', 20)         % outro ângulo inicial [graus do sensor]
%     pnmpc_hil('ref_x', @(t) 0.05*(t > 10))      % mover o carrinho já equilibrado
%
%   Antes: ident_preparar_volta (KAN + kan_planta_pesos.h), bancada_ajustar e gravar o
%   sketch esp32/kan_planta_esp32 no ESP32-S3.

cfg = config_padrao();
for i = 1:2:numel(varargin)
    if ~isfield(cfg, varargin{i}), error('Opção desconhecida: %s', varargin{i}); end
    cfg.(varargin{i}) = varargin{i+1};
end
aqui = fileparts(mfilename('fullpath'));
tmp = load(fullfile(aqui, cfg.arquivo_kan), 'kan'); kan = tmp.kan;
tmp = load(fullfile(aqui, 'bancada_param.mat'), 'p'); pp = tmp.p;
pp.quantizar = cfg.quantizar; pp.x_lim = cfg.x_lim;
planta = bancada_planta(pp);
mpc = cfg.mpc; if isempty(mpc), mpc = pnmpc_param(); end

Ts = kan.Ts;
N = round(cfg.Tfinal/Ts);
dados.cfg = cfg; dados.mpc = mpc; dados.p = pp; dados.Ts = Ts;
dados.t = (0:N)'*Ts;
dados.r = arrayfun(cfg.ref_x, dados.t);
dados.r_th = sensor2bancada(arrayfun(cfg.ref_th, dados.t));     % [rad], convenção da bancada
dados.Y = nan(N+1, 4); dados.X = nan(N+1, 4); dados.U = nan(N+1, 1);
dados.t_us = nan(N+1, 1); dados.t_troca_ms = nan(N+1, 1);
dados.modo = nan(N+1, 1); dados.E = nan(N+1, 1); dados.atrasou = false(N+1, 1);

sp = [];
if strcmpi(cfg.modo, 'hil')
    [sp, porta] = abrir_esp32(cfg, kan, mpc);
    fecha = onCleanup(@() fechar(sp)); %#ok<NASGU>
    nome = sprintf('PNMPC-KAN no ESP32-S3 (%s)', porta);
else
    nome = 'PNMPC-KAN no MATLAB (modo sim)';
end
dados.nome = nome;
est = [];

planta.reiniciar([cfg.x0 0 sensor2bancada(cfg.th0) 0]);
y = [planta.s(1) planta.s(2) planta.th_med planta.w_med];

gui = [];
if cfg.gui, gui = criar_gui(dados); end
fprintf('%s | Ts = %.0f ms | %.0f -> %.0f graus (sensor) | N = %d, %d iteração(ões)\n', ...
    nome, 1000*Ts, cfg.th0, cfg.ref_th(0), mpc.N, mpc.iters);

k_fim = N;
t0 = tic;
for k = 1:N
    r = dados.r(k); rth = dados.r_th(k);
    tq = tic;
    if isempty(sp)
        [u, est, info] = pnmpc_kan_passo(kan, mpc, est, y, r, rth);
        t_us = NaN; modo = info.modo; E = info.E;
    else
        fprintf(sp, '%s\n', sprintf('C,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g', k, y, r, rth));
        l = fgetl(sp);
        v = sscanf(l, 'U,%f,%f,%f,%f,%f,%f,%f,%f');
        if numel(v) < 8 || v(1) ~= k, error('Resposta inválida do ESP32 no passo %d: "%s"', k, l); end
        u = v(2); t_us = v(3); modo = v(7); E = v(8);
    end
    dados.t_troca_ms(k) = 1000*toc(tq);
    if ~isfinite(u), u = 0; end
    dados.Y(k, :) = y; dados.X(k, :) = planta.s; dados.U(k) = u;
    dados.t_us(k) = t_us; dados.modo(k) = modo; dados.E(k) = E;

    y = planta.passo(u, Ts);

    if abs(planta.s(1)) > cfg.x_lim
        warning('Carrinho passou do limite de %.2f m em t = %.2f s: parado.', cfg.x_lim, dados.t(k+1));
        k_fim = k; break;
    end
    if cfg.gui
        if ~ishandle(gui.fig) || getappdata(gui.fig, 'parar'), k_fim = k; break; end
        if mod(k, cfg.atualizar) == 0, atualizar_gui(gui, dados, k); end
    end
    if cfg.tempo_real
        if toc(t0) > k*Ts, dados.atrasou(k) = true; end
        while toc(t0) < k*Ts, end
    end
end
dados.Y(k_fim+1, :) = y; dados.X(k_fim+1, :) = planta.s;

idx = 1:k_fim+1;
for c = {'t', 'r', 'r_th', 'Y', 'X', 'U', 't_us', 't_troca_ms', 'modo', 'E', 'atrasou'}
    dados.(c{1}) = dados.(c{1})(idx, :);
end
dados.th_sensor = bancada2sensor(dados.X(:, 3));
dados.metricas = metricas(dados);
m = dados.metricas;
fprintf('\nResultado (%.1f s):\n', dados.t(end));
if isnan(m.t_captura)
    fprintf('  NÃO capturou o pêndulo em %.0f graus.\n', cfg.ref_th(0));
else
    fprintf('  captura (|erro| < 5 graus e sem voltar a passar de 20): t = %.2f s\n', m.t_captura);
    fprintf('  depois da captura: erro de ângulo RMS %.2f graus (máx %.2f) | x RMS %.1f mm\n', ...
        m.eth_rms, m.eth_max, 1000*m.ex_rms);
end
fprintf('  |x| máx %.3f m | a_cmd RMS %.2f m/s^2\n', m.x_max, m.u_rms);
tc = dados.t_us(~isnan(dados.t_us));
if ~isempty(tc), fprintf('  PNMPC no ESP32: média %.0f us, máx %.0f us\n', mean(tc), max(tc)); end
tt = dados.t_troca_ms(~isnan(dados.t_troca_ms));
fprintf('  troca por amostra: média %.2f ms; %d passos atrasados (Ts = %.0f ms)\n', mean(tt), sum(dados.atrasou), 1000*Ts);
if ~isempty(cfg.salvar), save(cfg.salvar, 'dados'); fprintf('Dados salvos em %s\n', cfg.salvar); end
if cfg.gui && ishandle(gui.fig), atualizar_gui(gui, dados, numel(idx) - 1, true); end
end

% =====================================================================
function cfg = config_padrao()
cfg.modo       = 'hil';            % 'hil' (PNMPC no ESP32) ou 'sim' (PNMPC no MATLAB)
cfg.porta      = 'COM8';
cfg.baud       = 921600;
cfg.timeout    = 2;
cfg.Tfinal     = 15;               % [s]
cfg.th0        = 210;              % ângulo inicial [graus do sensor: 90 = em pé, 270 = pendurado]
cfg.ref_th     = @(t) 90;          % referência de ângulo [graus do sensor]
cfg.ref_x      = @(t) 0;           % referência de posição do carrinho [m]
cfg.x0         = 0;                % [m]
cfg.x_lim      = 0.20;             % fim do trilho na simulação [m]
cfg.quantizar  = true;             % encoder de 0,15 grau como na bancada
cfg.arquivo_kan = 'kan_volta.mat';
cfg.mpc        = [];               % struct de pnmpc_param (vazio = padrão)
cfg.tempo_real = true;
cfg.gui        = true;
cfg.atualizar  = 5;
cfg.janela     = 15;               % [s] nos gráficos ao vivo
cfg.salvar     = 'resultado_pnmpc_kan.mat';
end

function th = sensor2bancada(g)
th = (g + 90)*pi/180;              % sensor 90 (em pé) -> pi ; 270 (pendurado) -> 2*pi
end

function g = bancada2sensor(th)
g = mod(th*180/pi - 90, 360);
end

function m = metricas(d)
eth = (d.X(:, 3) - d.r_th); eth = (eth - 2*pi*floor((eth + pi)/(2*pi)))*180/pi;
m.t_captura = NaN;
ok = abs(eth) < 5; ruim = abs(eth) > 20;
ult_ruim = find(ruim, 1, 'last'); if isempty(ult_ruim), ult_ruim = 0; end
i = find(ok((ult_ruim+1):end), 1) + ult_ruim;
if ~isempty(i) && ult_ruim < numel(eth), m.t_captura = d.t(i); end
if ~isnan(m.t_captura)
    sel = d.t >= m.t_captura;
    m.eth_rms = sqrt(mean(eth(sel).^2)); m.eth_max = max(abs(eth(sel)));
    m.ex_rms = sqrt(mean((d.X(sel, 1) - d.r(sel)).^2));
else
    m.eth_rms = NaN; m.eth_max = NaN; m.ex_rms = NaN;
end
m.x_max = max(abs(d.X(:, 1)));
m.u_rms = sqrt(mean(d.U(~isnan(d.U)).^2));
end

% =====================================================================
function gui = criar_gui(d)
gui.fig = figure('Name', ['PNMPC-KAN HIL - ' d.nome], 'Color', 'w', 'NumberTitle', 'off', ...
    'Position', [40 40 1300 820]);
setappdata(gui.fig, 'parar', false);
uicontrol(gui.fig, 'Style', 'pushbutton', 'String', 'Parar', 'Units', 'normalized', ...
    'Position', [0.92 0.96 0.07 0.032], 'Callback', @(h, ~) setappdata(gui.fig, 'parar', true));
gui.ax(1) = subplot(5, 2, [1 2]);
gui.lrt = plot(nan, nan, 'k--', 'LineWidth', 1.2); hold on;
gui.lth = plot(nan, nan, 'r', 'LineWidth', 1.5);
ylabel('\theta sensor [graus]'); grid on; ylim([0 360]); set(gca, 'YTick', 0:90:360);
legend('referência', '\theta (90 = em pé, 270 = pendurado)', 'Location', 'southeast');
gui.tit = title(d.nome);
gui.ax(2) = subplot(5, 2, [3 4]);
gui.lr = stairs(nan, nan, 'k--', 'LineWidth', 1.2); hold on;
gui.lx = plot(nan, nan, 'b', 'LineWidth', 1.5);
plot([0 1e6], d.cfg.x_lim*[1 1], 'r:', [0 1e6], -d.cfg.x_lim*[1 1], 'r:');
ylabel('x [m]'); grid on; ylim(1.15*d.cfg.x_lim*[-1 1]);
gui.ax(3) = subplot(5, 2, [5 6]);
gui.lu = stairs(nan, nan, 'Color', [0 0.5 0], 'LineWidth', 1.2);
ylabel('a_{cmd} [m/s^2]'); grid on; ylim(1.15*[d.mpc.u_min d.mpc.u_max]);
gui.ax(4) = subplot(5, 2, [7 8]);
gui.lE = plot(nan, nan, 'm', 'LineWidth', 1.2); hold on;
plot([0 1e6], d.mpc.E_alvo*[1 1], 'k:');
gui.lm = stairs(nan, nan, 'Color', [0.3 0.3 0.3], 'LineWidth', 1.0);
ylabel('energia / modo'); grid on; ylim([-1.3 2]);
legend('E (-1 pendurado, +1 em pé)', 'E alvo', 'modo (0 swing-up, 1 equilíbrio)', 'Location', 'southeast');
gui.ax(5) = subplot(5, 2, [9 10]);
gui.lt = plot(nan, nan, 'Color', [0.6 0 0.6], 'LineWidth', 1.0);
ylabel('cálculo ESP32 [us]'); xlabel('t [s]'); grid on;
gui.janela = d.cfg.janela;
drawnow;
end

function atualizar_gui(gui, d, n, final)
if nargin < 4, final = false; end
t = d.t(1:n);
if final, r = 1:n; else, r = find(t >= t(end) - gui.janela, 1):n; end
ths = bancada2sensor(d.X(r, 3)); ths(abs(diff([ths(1); ths])) > 180) = nan;   % sem traço na volta 0/360
set(gui.lth, 'XData', t(r), 'YData', ths);
set(gui.lrt, 'XData', t(r), 'YData', bancada2sensor(d.r_th(r)));
set(gui.lr, 'XData', t(r), 'YData', d.r(r));
set(gui.lx, 'XData', t(r), 'YData', d.X(r, 1));
set(gui.lu, 'XData', t(r), 'YData', d.U(r));
set(gui.lE, 'XData', t(r), 'YData', d.E(r));
set(gui.lm, 'XData', t(r), 'YData', d.modo(r));
set(gui.lt, 'XData', t(r), 'YData', d.t_us(r));
lim = [t(r(1)), max(t(r(end)), t(r(1)) + 1e-3)];
if ~final, lim(2) = max(lim(2), lim(1) + gui.janela); end
for j = 1:5, xlim(gui.ax(j), lim); end
md = {'SWING-UP', 'EQUILÍBRIO'};
set(gui.tit, 'String', sprintf('%s | t = %.1f s | \\theta = %.1f graus | %s | cálculo %s us', ...
    d.nome, t(end), bancada2sensor(d.X(n, 3)), md{1 + (d.modo(n) == 1)}, num2str(d.t_us(n), '%.0f')));
if final, drawnow; else, drawnow limitrate; end
end

% =====================================================================
function [sp, porta] = abrir_esp32(cfg, kan, mpc)
porta = cfg.porta;
velhas = instrfind('Port', porta);
if ~isempty(velhas), fclose(velhas); delete(velhas); end
sp = serial(porta, 'BaudRate', cfg.baud, 'Terminator', 'LF', 'Timeout', 0.5, ...
    'InputBufferSize', 4096, 'OutputBufferSize', 1024, 'DataTerminalReady', 'off', 'RequestToSend', 'off');
fopen(sp); pause(0.3); flushinput(sp);
info = [];
for tent = 1:8
    fprintf(sp, '%s\n', '?');
    l = fgetl(sp);
    if ischar(l) && strncmp(strtrim(l), 'PRONTO,KAN_PLANTA_ESP32', 23)
        partes = regexp(strtrim(l), ',', 'split');
        info = str2double(partes(4:end));
        break;
    end
    pause(0.3); flushinput(sp);
end
if isempty(info)
    fclose(sp); delete(sp);
    error('O ESP32 não respondeu ao "?" em %s. Feche outros programas na porta e aperte RESET.', porta);
end
ass = ident_kan_assinatura(kan);
if numel(info) < 3 || isnan(info(3)) || abs(info(3) - ass) > 1e-5*max(1, ass)
    fclose(sp); delete(sp);
    error('A KAN gravada no ESP32 não é a de %s: gere o .h e grave o sketch de novo.', cfg.arquivo_kan);
end
set(sp, 'Timeout', cfg.timeout);
par = [mpc.q mpc.q_term mpc.r_du mpc.r_u mpc.u_min mpc.u_max mpc.eps mpc.alfa_n mpc.iters ...
       mpc.varreduras mpc.q_E mpc.E_alvo mpc.w0q mpc.phi_sw mpc.q_sw_x mpc.q_sw_v mpc.q_xs mpc.x_soft ...
       mpc.Pt mpc.paralelo mpc.v_max mpc.q_vs mpc.v_soft];
msg = ['P' sprintf(',%.9g', par)];
fprintf(sp, '%s\n', msg);
l = strtrim(fgetl(sp));
v = sscanf(l, 'OK,P,%f,%f');
if isempty(v), fclose(sp); delete(sp); error('O ESP32 recusou a sintonia: "%s"', l); end
if v(1) ~= mpc.N || v(2) ~= numel(mpc.bloco)
    fclose(sp); delete(sp);
    error('Horizonte do firmware (N = %d, %d movimentos) difere de pnmpc_param.', v(1), v(2));
end
fprintf(sp, '%s\n', 'Z'); l = strtrim(fgetl(sp));
if ~strncmp(l, 'OK', 2), error('Resposta inesperada ao Z: "%s"', l); end
fprintf('ESP32 pronto em %s: KAN confere, sintonia enviada.\n', porta);
end

function fechar(sp)
try, fclose(sp); catch, end
try, delete(sp); catch, end
end
