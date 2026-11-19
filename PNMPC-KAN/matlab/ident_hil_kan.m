function dados = ident_hil_kan(varargin)
%IDENT_HIL_KAN  HIL de identificação: planta no MATLAB, KAN da planta no ESP32.
%
%   A planta não linear (carrinho + pêndulo PENDURADO) roda aqui em MALHA ABERTA,
%   excitada por um trem de pulsos na força. A cada Ts o MATLAB manda ao ESP32 o
%   estado medido e a força; o ESP32 devolve, calculado pela KAN em um núcleo
%   dedicado, o estado previsto para o próximo instante. Os gráficos mostram, ao vivo,
%   a planta e as duas previsões da KAN:
%     - um passo à frente  (KAN recebe o estado real a cada amostra)
%     - simulação livre     (KAN realimenta a própria saída, sem ver a planta)
%
%   Antes: rode ident_preparar_kan (treina e gera kan_planta_pesos.h) e carregue
%   esp32/kan_planta_esp32/kan_planta_esp32.ino no ESP32.
%
%   Uso:
%     ident_hil_kan                                  % modo 'sim': KAN no próprio MATLAB
%     ident_hil_kan('modo', 'hil', 'porta', 'COM5')  % KAN no ESP32
%     d = ident_hil_kan('modo','hil','porta','COM5','Tfinal',60);
%     s = struct('tipo','regular','A',1.5,'largura',0.1,'periodo',1,'atraso',0.5);
%     ident_hil_kan('modo','hil','porta','COM5','sinal',s)      % outro trem de pulsos
%
%   PLANTA DA BANCADA (ensaio gravado): em vez de simular, o MATLAB REPRODUZ em tempo
%   real um ensaio da bancada (estado medido + a_cmd) e o ESP32 prevê o passo seguinte.
%   A KAN deve ter sido treinada com ident_preparar_bancada (kan_bancada.mat):
%     ident_hil_kan('dados', '..\Planta_smiluação\ensaio_20261119_154800.csv')
%     ident_hil_kan('dados', 'bancada')            % atalho para o ensaio 154800
%
%   Opções -> veja config_padrao() no fim deste arquivo.

cfg = config_padrao();
for i = 1:2:numel(varargin)
    if ~isfield(cfg, varargin{i}), error('Opção desconhecida: %s', varargin{i}); end
    cfg.(varargin{i}) = varargin{i+1};
    if strcmp(varargin{i}, 'Tfinal'), cfg.Tfinal_padrao = false; end
end
aqui = fileparts(mfilename('fullpath'));
replay = ~isempty(cfg.dados);
if replay
    if strcmpi(cfg.dados, 'bancada')
        cfg.dados = fullfile(aqui, '..', 'Planta_smiluação', 'ensaio_20261119_154800.csv');
    end
    if strcmp(cfg.arquivo_kan, 'kan_planta.mat'), cfg.arquivo_kan = 'kan_bancada.mat'; end
end
arq = cfg.arquivo_kan;
if ~exist(arq, 'file'), arq = fullfile(aqui, cfg.arquivo_kan); end
if ~exist(arq, 'file')
    error('Não achei %s. Rode ident_preparar_kan (planta simulada) ou ident_preparar_bancada (ensaio) primeiro.', cfg.arquivo_kan);
end
tmp = load(arq, 'kan'); kan = tmp.kan;
if isempty(cfg.sinal)
    if isfield(kan, 'sinal_ensaio'), cfg.sinal = kan.sinal_ensaio;
    else, cfg.sinal = struct('tipo', 'regular'); end
end

p = parametros_planta();
Ts = kan.Ts;                       % a KAN só vale para o Ts do treino
x_lim = p.L_trilho/2;
if ~isfield(kan, 'th_eq'), kan.th_eq = pi; end
if replay
    % ensaio gravado: estado e entrada vêm do arquivo, amostra a amostra
    if isstruct(cfg.dados), Dg = cfg.dados; else, Dg = ident_ler_bancada(cfg.dados); end
    ok = Dg.serie.seg >= 0;
    r = find(ok, 1):find(ok, 1, 'last');
    if ~isinf(cfg.Tfinal) && cfg.Tfinal < 1e3 && ~cfg.Tfinal_padrao
        r = r(1:min(end, round(cfg.Tfinal/Ts) + 1));
    end
    S_grav = Dg.serie.S(r, :);
    N = numel(r) - 1;
    cfg.x0 = S_grav(1, :);
    dados.t = Dg.serie.t(r) - Dg.serie.t(r(1));
    dados.U = Dg.serie.U(r);
    cfg.Tfinal = dados.t(end);
    rot_u = 'a_{cmd} [m/s^2]';
    if isfield(kan, 'rotulo_u'), rot_u = kan.rotulo_u; end
else
    N = round(cfg.Tfinal/Ts);
    dados.t  = (0:N)'*Ts;
    dados.U  = ident_trem_pulsos(dados.t, cfg.sinal);
    rot_u = 'u [N]';
end

dados.cfg = cfg; dados.p = p; dados.Ts = Ts;
dados.th_eq = kan.th_eq; dados.rotulo_u = rot_u;
dados.S  = nan(N+1, 4);            % planta (verdade)
dados.P1 = nan(N+1, 4);            % KAN um passo à frente
dados.PL = nan(N+1, 4);            % KAN simulação livre
dados.t_calc_us  = nan(N+1, 1);    % tempo de cálculo da KAN no ESP32
dados.t_troca_ms = nan(N+1, 1);    % envio + planta + resposta
dados.atrasou    = false(N+1, 1);  % passo que passou de Ts

% ---------------- ESP32 ----------------
sp = [];
if strcmpi(cfg.modo, 'hil')
    [sp, porta, info] = hil_abrir(cfg);
    fecha = onCleanup(@() hil_fechar(sp)); %#ok<NASGU>
    if numel(info) >= 1 && abs(info(1) - Ts) > 1e-6
        error('Ts do ESP32 (%.4f s) difere do Ts da KAN no MATLAB (%.4f s): gere o .h de novo.', info(1), Ts);
    end
    if numel(info) >= 2
        fprintf('Autoteste no ESP32: erro máximo float x MATLAB = %.2e\n', info(2));
        if info(2) > 1e-3
            warning('Autoteste alto: o kan_planta_pesos.h no ESP32 pode não ser o desta KAN.');
        end
    end
    ass = ident_kan_assinatura(kan);
    if numel(info) >= 3 && ~isnan(info(3))
        if abs(info(3) - ass) > 1e-5*max(1, abs(ass))
            error(['A KAN gravada no ESP32 não é a de %s (assinatura %.6g no ESP32 x %.6g aqui).\n' ...
                   'Gere o .h com a KAN certa (ident_exportar_esp32 / ident_preparar_*) e grave o sketch de novo.'], ...
                   arq, info(3), ass);
        end
        fprintf('KAN no ESP32 confere com %s.\n', arq);
    else
        warning('O ESP32 não informou a assinatura da KAN (sketch antigo?): não deu para conferir se é a mesma rede.');
    end
    nome = sprintf('KAN no ESP32 (%s)', porta);
elseif strcmpi(cfg.modo, 'sim')
    nome = 'KAN no MATLAB (modo sim)';
else
    error('modo deve ser ''sim'' ou ''hil''.');
end

s = cfg.x0(:);
dados.S(1, :) = s.'; dados.P1(1, :) = s.'; dados.PL(1, :) = s.';
if isempty(sp)
    sl = s.';
else
    l = hil_trocar(sp, sprintf('R,%.9g,%.9g,%.9g,%.9g', s));
    if ~strncmp(l, 'OK', 2), error('Resposta inesperada ao R: "%s"', l); end
    fprintf('Simulação livre reiniciada no ESP32 (KAN no núcleo %s).\n', strtrim(l(4:end)));
end

gui = [];
if cfg.gui, gui = criar_gui(dados, nome); end

% ---------------- laço em tempo real ----------------
fprintf('Ensaio de %.0f s, Ts = %.0f ms, %s\n', cfg.Tfinal, 1000*Ts, nome);
k_fim = N;
t0 = tic;
for k = 1:N
    y = s + cfg.ruido_med(:).*randn(4, 1);       % estado medido mandado à KAN
    u = dados.U(k);
    tq = tic;
    if isempty(sp)
        p1 = ident_kan_prever(kan, y.', u);
        sl = ident_kan_prever(kan, sl, u);
        pl = sl; t_us = NaN;
    else
        hil_enviar(sp, sprintf('S,%d,%.7g,%.9g,%.9g,%.9g,%.9g', k, u, y));
    end
    if replay
        s = S_grav(k+1, :).';                              % ensaio gravado: próxima amostra
    else
        s = ident_planta_passo(s, u, Ts, p, cfg.subpassos);    % planta enquanto o ESP32 calcula
    end
    if ~isempty(sp)
        l = hil_ler(sp);
        v = sscanf(l, 'P,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
        if numel(v) < 10 || v(1) ~= k
            error('Resposta inválida do ESP32 no passo %d: "%s"', k, l);
        end
        p1 = v(2:5).'; pl = v(6:9).'; t_us = v(10);
    end
    dados.t_troca_ms(k+1) = 1000*toc(tq);
    dados.S(k+1, :) = s.'; dados.P1(k+1, :) = p1; dados.PL(k+1, :) = pl;
    dados.t_calc_us(k+1) = t_us;

    if ~replay && abs(s(1)) > x_lim
        warning('Carrinho chegou ao fim do trilho (x = %.3f m) em t = %.2f s. Reduza A/largura/período do trem de pulsos.', ...
            s(1), dados.t(k+1));
        k_fim = k; break;
    end
    if cfg.gui
        if ~ishandle(gui.fig) || getappdata(gui.fig, 'parar'), k_fim = k; break; end
        if mod(k, cfg.atualizar) == 0, atualizar_gui(gui, dados, k+1); end
    end
    if cfg.tempo_real
        if toc(t0) > k*Ts, dados.atrasou(k+1) = true; end
        while toc(t0) < k*Ts, end                 % espera ativa: pause() é grosseiro no Windows
    end
end

% ---------------- resultados ----------------
idx = 1:k_fim+1;
nomes_c = {'t', 'U', 'S', 'P1', 'PL', 't_calc_us', 't_troca_ms', 'atrasou'};
for i = 1:numel(nomes_c), dados.(nomes_c{i}) = dados.(nomes_c{i})(idx, :); end
e1 = dados.P1(2:end, :) - dados.S(2:end, :);
eL = dados.PL(2:end, :) - dados.S(2:end, :);
dados.rmse_1p = sqrt(mean(e1.^2, 1));
dados.rmse_livre = sqrt(mean(eL.^2, 1));
dados.nome = nome;
esc = [1000 1000 180/pi 180/pi];
fprintf('\nResultado (%d amostras):\n', k_fim);
fprintf('                 %10s %10s %10s %10s\n', 'x [mm]', 'dx [mm/s]', 'th [graus]', 'dth [g/s]');
fprintf('  RMSE 1 passo   %10.4f %10.4f %10.4f %10.4f\n', dados.rmse_1p.*esc);
fprintf('  RMSE livre     %10.4f %10.4f %10.4f %10.4f\n', dados.rmse_livre.*esc);
if ~isempty(sp)
    fprintf('  cálculo da KAN no ESP32: média %.0f us, máx %.0f us\n', ...
        mean(dados.t_calc_us(2:end)), max(dados.t_calc_us(2:end)));
end
fprintf('  troca por amostra: média %.2f ms, máx %.2f ms (Ts = %.0f ms); %d passos atrasados\n', ...
    mean(dados.t_troca_ms(2:end)), max(dados.t_troca_ms(2:end)), 1000*Ts, sum(dados.atrasou));

if ~isempty(cfg.salvar)
    save(cfg.salvar, 'dados');
    fprintf('Dados salvos em %s\n', cfg.salvar);
end
if cfg.gui && ishandle(gui.fig), atualizar_gui(gui, dados, numel(idx), true); end
if cfg.gui, ident_plotar_erro(dados); end           % erro da KAN ao longo do tempo
end

% =====================================================================
function cfg = config_padrao()
cfg.modo        = 'hil';       % 'sim' (KAN no MATLAB) ou 'hil' (KAN no ESP32)
cfg.porta       = 'COM8';          % ex.: 'COM5'; vazio = última porta disponível
cfg.baud        = 921600;      % igual a BAUD no sketch
cfg.resetar_placa = false;   % true só para ESP32 com conversor USB-serial (CP2102/CH340)
cfg.timeout     = 1;           % [s] espera da resposta
cfg.Tfinal      = 30;          % [s] (com 'dados': padrão = o ensaio inteiro)
cfg.Tfinal_padrao = true;      % (interno) vira false quando o usuário passa 'Tfinal'
cfg.dados       = '';          % ensaio da bancada (CSV/XLSX ou 'bancada'): reproduz em vez de simular
cfg.sinal       = [];          % trem de pulsos (vazio = o da validação em kan_planta.mat)
cfg.x0          = [0 0 pi 0];  % carrinho no meio, pêndulo pendurado e parado
cfg.ruido_med   = [0 0 0 0];   % desvio-padrão do ruído no estado mandado à KAN
cfg.subpassos   = 4;           % RK4 internos por Ts
cfg.tempo_real  = true;
cfg.gui         = true;
cfg.atualizar   = 5;           % redesenha a cada n amostras
cfg.janela      = 10;          % [s] mostrados nos gráficos ao vivo
cfg.arquivo_kan = 'kan_planta.mat';
cfg.salvar      = 'resultado_ident_kan.mat';
end

% =====================================================================
function gui = criar_gui(dados, nome)
gui.fig = figure('Name', ['Identificação HIL - ' nome], 'Color', 'w', ...
    'NumberTitle', 'off', 'Position', [30 40 1480 800]);
setappdata(gui.fig, 'parar', false);
uicontrol(gui.fig, 'Style', 'pushbutton', 'String', 'Parar', 'Units', 'normalized', ...
    'Position', [0.92 0.96 0.07 0.032], 'Callback', @(h, ~) setappdata(gui.fig, 'parar', true));
% coluna da esquerda: estados (planta x KAN) | coluna da direita: erro 1 passo ao vivo
if dados.th_eq == 0                 % bancada: 0 = pendurado
    rot = {'x [m]', 'v [m/s]', '\theta [graus]', '\omega [graus/s]'};
else
    rot = {'x [m]', 'dx [m/s]', '\theta - \pi [graus]', 'd\theta/dt [graus/s]'};
end
rotE = {'erro x [mm]', 'erro dx [mm/s]', 'erro \theta [graus]', 'erro d\theta/dt [graus/s]'};
for j = 1:4
    gui.ax(j) = subplot(5, 2, 2*j - 1);
    gui.lS(j)  = plot(nan, nan, 'k', 'LineWidth', 1.5); hold on;
    gui.l1(j)  = plot(nan, nan, 'r--', 'LineWidth', 1.1);
    gui.lL(j)  = plot(nan, nan, 'b:', 'LineWidth', 1.3);
    ylabel(rot{j}); grid on;

    gui.axE(j) = subplot(5, 2, 2*j);
    gui.lE(j) = plot(nan, nan, 'r', 'LineWidth', 1.0); hold on;
    plot([0 1e6], [0 0], 'k:');
    ylabel(rotE{j}); grid on;
    gui.titE(j) = title(gui.axE(j), '', 'FontWeight', 'normal');
end
legend(gui.ax(1), 'planta (MATLAB)', 'KAN 1 passo', 'KAN livre', 'Location', 'northwest', 'Orientation', 'horizontal');
gui.tit = title(gui.ax(1), nome);
gui.ax(5) = subplot(5, 2, 9);
gui.lU = stairs(nan, nan, 'Color', [0 0.5 0], 'LineWidth', 1.3);
ylabel(dados.rotulo_u); xlabel('t [s]'); grid on;
% erro da simulação livre em theta (a KAN sem ver a planta)
gui.axE(5) = subplot(5, 2, 10);
gui.lE(5) = plot(nan, nan, 'b', 'LineWidth', 1.0); hold on;
plot([0 1e6], [0 0], 'k:');
ylabel('erro livre \theta [graus]'); xlabel('t [s]'); grid on;
gui.titE(5) = title(gui.axE(5), '', 'FontWeight', 'normal');
A = max(abs(dados.U)); if A == 0, A = 1; end
ylim(gui.ax(5), 1.2*[-A A]);
gui.janela = dados.cfg.janela;
drawnow;
end

function atualizar_gui(gui, dados, n, final)
if nargin < 4, final = false; end
esc = [1 1 180/pi 180/pi]; desl = [0 0 dados.th_eq 0];
t = dados.t(1:n);
if final, ini = 1; else, ini = find(t >= t(end) - gui.janela, 1); end
r = ini:n;
for j = 1:4
    set(gui.lS(j), 'XData', t(r), 'YData', (dados.S(r, j) - desl(j))*esc(j));
    set(gui.l1(j), 'XData', t(r), 'YData', (dados.P1(r, j) - desl(j))*esc(j));
    set(gui.lL(j), 'XData', t(r), 'YData', (dados.PL(r, j) - desl(j))*esc(j));
end
set(gui.lU, 'XData', t(r), 'YData', dados.U(r));
% ---- erro ao vivo: previsão - planta ----
escE = [1000 1000 180/pi 180/pi];
rr = r(r >= 2);                                  % amostra 1 não tem previsão
for j = 1:4
    e = (dados.P1(rr, j) - dados.S(rr, j))*escE(j);
    set(gui.lE(j), 'XData', t(rr), 'YData', e);
    eT = (dados.P1(2:n, j) - dados.S(2:n, j))*escE(j);   % desde o início
    set(gui.titE(j), 'String', sprintf('1 passo: RMSE %.3g | máx %.3g', ...
        sqrt(mean(eT.^2)), max(abs(eT))));
end
eL = (dados.PL(rr, 3) - dados.S(rr, 3))*180/pi;
set(gui.lE(5), 'XData', t(rr), 'YData', eL);
eLT = (dados.PL(2:n, 3) - dados.S(2:n, 3))*180/pi;
set(gui.titE(5), 'String', sprintf('simulação livre: RMSE %.3g | máx %.3g', ...
    sqrt(mean(eLT.^2)), max(abs(eLT))));
lim = [t(r(1)), max(t(r(end)), t(r(1)) + 1e-3)];
if ~final, lim(2) = max(lim(2), lim(1) + gui.janela); end
for j = 1:5, xlim(gui.ax(j), lim); xlim(gui.axE(j), lim); end
tc = dados.t_calc_us(n);
set(gui.tit, 'String', sprintf('t = %.2f s | cálculo da KAN: %s us', t(end), num2str(tc, '%.0f')));
if final, drawnow; else, drawnow limitrate; end
end

% =====================================================================
function [sp, porta, info] = hil_abrir(cfg)
porta = cfg.porta;
if isempty(porta)
    try
        inf_ = instrhwinfo('serial'); lst = inf_.AvailableSerialPorts;
    catch
        lst = {};
    end
    if isempty(lst), error('Nenhuma porta serial disponível. Conecte o ESP32 ou informe ''porta''.'); end
    porta = lst{end};
    fprintf('Usando a porta %s\n', porta);
end
velhas = instrfind('Port', porta);
if ~isempty(velhas), fclose(velhas); delete(velhas); end
sp = serial(porta, 'BaudRate', cfg.baud, 'Terminator', 'LF', 'Timeout', 0.5, ...
    'InputBufferSize', 4096, 'OutputBufferSize', 1024, ...
    'DataTerminalReady', 'off', 'RequestToSend', 'off');
fopen(sp);
% Reset limpo no modo normal (circuito de auto-reset das placas ESP32 DevKit):
% RTS ligado = EN em 0 (reset); DTR desligado = GPIO0 em 1 (roda o programa, não grava).
if cfg.resetar_placa      % pulso RTS: reset de ESP32 com conversor CP2102/CH340
    set(sp, 'DataTerminalReady', 'off', 'RequestToSend', 'on'); pause(0.1);
    set(sp, 'RequestToSend', 'off');
    fprintf('Aguardando o ESP32 reiniciar...\n');
    pause(1.5);
else
    pause(0.3);              % USB nativo (ESP32-S3): sem reset, o MATLAB manda R
end
flushinput(sp);
info = [];
recebido = '';
for tent = 1:8
    fprintf(sp, '%s\n', '?');
    l = hil_ler(sp, true);
    if ~isempty(l), recebido = l; end
    if strncmp(l, 'PRONTO,KAN_PLANTA_ESP32', 23)
        partes = regexp(l, ',', 'split');
        info = str2double(partes(4:end));
        break;
    end
    pause(0.3);
    if sp.BytesAvailable > 0
        resto = fread(sp, sp.BytesAvailable, 'char')';
        recebido = char(resto(resto >= 32 & resto < 127));
    end
end
if isempty(info)
    fclose(sp); delete(sp);
    if isempty(recebido)
        dica = ['nada chegou pela serial: confira a porta (' porta '), se o sketch foi carregado ' ...
                'e aperte EN/RESET na placa logo antes de rodar'];
    else
        dica = ['chegou "' recebido(1:min(end, 60)) '": se for lixo, o baud nao bate ' ...
                '(BAUD no sketch = ''baud'' aqui; tente 460800 ou 115200 nos dois)'];
    end
    error('O ESP32 nao respondeu ao "?" (baud %d): %s.', cfg.baud, dica);
end
set(sp, 'Timeout', cfg.timeout);
fprintf('ESP32 pronto em %s.\n', porta);
end

function hil_enviar(sp, msg)
fprintf(sp, '%s\n', msg);
end

function l = hil_ler(sp, tolerante)
if nargin < 2, tolerante = false; end
l = fgetl(sp);
if ~ischar(l) || isempty(l)
    if tolerante, l = ''; return; end
    error('O ESP32 não respondeu dentro do timeout.');
end
l = strtrim(l);
end

function l = hil_trocar(sp, msg)
hil_enviar(sp, msg);
l = hil_ler(sp);
end

function hil_fechar(sp)
if isempty(sp), return; end
try, fclose(sp); catch, end
try, delete(sp); catch, end
end
