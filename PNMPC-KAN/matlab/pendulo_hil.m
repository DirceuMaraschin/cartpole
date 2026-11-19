function dados = pendulo_hil(varargin)
%PENDULO_HIL  Hardware-in-the-loop do pêndulo invertido sobre carrinho.
%
%   A PLANTA (carrinho + pêndulo, não linear) é simulada aqui no MATLAB.
%   O CONTROLADOR roda no Arduino: a cada período Ts o MATLAB manda o estado
%   medido pela serial e o Arduino devolve a força u. Há também o modo 'sim',
%   em que o controlador roda no próprio MATLAB (para testar sem a placa).
%
%   Uso:
%     pendulo_hil                                   % modo 'sim', LQR provisório
%     pendulo_hil('controlador', 'rede')            % modo 'sim', MLP (rede_exemplo.m)
%     pendulo_hil('controlador', 'kan')             % modo 'sim', KAN (kan_pesos.m)
%     pendulo_hil('modo', 'hil', 'porta', 'COM3')   % controlador no Arduino
%     dados = pendulo_hil('modo','hil','porta','/dev/ttyACM0','Tfinal',60);
%
%   Opções (pares nome/valor) -> veja config_padrao() no fim deste arquivo.
%
%   Na janela: slider de força contínua no carrinho, slider + botão de
%   empurrão na ponta da haste, pausa e parada. Ao final são plotadas as
%   respostas completas e os dados são salvos em resultado_pendulo.mat.

cfg = config_padrao();
for i = 1:2:numel(varargin)
    if ~isfield(cfg, varargin{i}), error('Opção desconhecida: %s', varargin{i}); end
    cfg.(varargin{i}) = varargin{i+1};
end
p = parametros_planta();
if ~isempty(cfg.planta)                       % sobrescreve parâmetros
    nomes = fieldnames(cfg.planta);
    for i = 1:numel(nomes), p.(nomes{i}) = cfg.planta.(nomes{i}); end
end

Ts = cfg.Ts;
N  = round(cfg.Tfinal/Ts);
h  = Ts/cfg.subpassos;
x_lim = p.L_trilho/2;

% ---------------- controlador ----------------
sp = [];
switch lower(cfg.modo)
    case 'hil'
        [sp, cfg.porta_usada] = hil_abrir(cfg);
        fecha = onCleanup(@() hil_fechar(sp)); %#ok<NASGU>
        nome_ctrl = sprintf('Arduino (%s)', cfg.porta_usada);
    case 'sim'
        if strcmpi(cfg.controlador, 'rede')
            if isempty(cfg.rede), cfg.rede = rede_exemplo(); end
            nome_ctrl = 'rede neural (MLP) no MATLAB';
        elseif strcmpi(cfg.controlador, 'kan')
            if isempty(cfg.kan), cfg.kan = kan_pesos(); end
            nome_ctrl = 'KAN no MATLAB';
        else
            nome_ctrl = 'LQR provisório no MATLAB';
        end
    otherwise
        error('modo deve ser ''sim'' ou ''hil''.');
end

% ---------------- armazenamento ----------------
dados.cfg = cfg; dados.p = p;
dados.t        = (0:N-1)'*Ts;
dados.X        = nan(N, 4);   % estado verdadeiro [x dx th dth] no instante t
dados.Y        = nan(N, 4);   % estado medido enviado ao controlador
dados.u_ctrl   = nan(N, 1);   % força pedida pelo controlador
dados.u        = nan(N, 1);   % força aplicada (após saturação)
dados.F_carro  = nan(N, 1);
dados.F_ponta  = nan(N, 1);
dados.t_calc_us = nan(N, 1);  % tempo de cálculo informado pelo Arduino
dados.t_troca_ms = nan(N, 1); % ida e volta pela serial
dados.fim_curso = false(N, 1);
dados.caiu      = false(N, 1);
dados.nome_ctrl = nome_ctrl;

% ---------------- interface ----------------
gui = [];
if cfg.gui
    gui = criar_gui(p, cfg, nome_ctrl);
end

s = cfg.x0(:);
t_fim_empurrao = -inf; amp_empurrao = 0;
k_fim = N;
t_ref = tic; t_pausa = 0;
for k = 1:N
    t = dados.t(k);

    % ---- perturbações ----
    F_c = 0; F_p = 0;
    if ~isempty(cfg.perturbacao)
        d = cfg.perturbacao(t); F_c = d(1); F_p = d(2);
    end
    if cfg.gui
        if ~ishandle(gui.fig), k_fim = k-1; break; end
        if getappdata(gui.fig, 'parar'), k_fim = k-1; break; end
        while ishandle(gui.fig) && getappdata(gui.fig, 'pausado') && ~getappdata(gui.fig, 'parar')
            tp = tic; pause(0.05); t_pausa = t_pausa + toc(tp);
        end
        if ~ishandle(gui.fig), k_fim = k-1; break; end
        F_c = F_c + getappdata(gui.fig, 'F_carro');
        if getappdata(gui.fig, 'pedido_empurrao')
            setappdata(gui.fig, 'pedido_empurrao', false);
            t_fim_empurrao = t + cfg.dur_empurrao;
            amp_empurrao = getappdata(gui.fig, 'F_ponta');
        end
        if t < t_fim_empurrao, F_p = F_p + amp_empurrao; end
    end

    % ---- medição (com ruído opcional) ----
    y = s + cfg.ruido_med(:).*randn(4, 1);

    % ---- controlador ----
    if isempty(sp)
        if strcmpi(cfg.controlador, 'rede')
            u_c = rede_forward(cfg.rede, y);
        elseif strcmpi(cfg.controlador, 'kan')
            u_c = kan_forward(cfg.kan, y);
        else
            u_c = -cfg.K(:).'*y;
        end
    else
        tt = tic;
        [u_c, dados.t_calc_us(k)] = hil_trocar(sp, y);
        dados.t_troca_ms(k) = 1e3*toc(tt);
    end
    u = min(max(u_c, -p.F_max), p.F_max);

    dados.X(k, :) = s.'; dados.Y(k, :) = y.';
    dados.u_ctrl(k) = u_c; dados.u(k) = u;
    dados.F_carro(k) = F_c; dados.F_ponta(k) = F_p;

    % ---- planta: RK4 com u constante no período (ZOH) ----
    f = @(ss) dinamica_pendulo(ss, u, F_c, F_p, p);
    for j = 1:cfg.subpassos
        k1 = f(s); k2 = f(s + h/2*k1); k3 = f(s + h/2*k2); k4 = f(s + h*k3);
        s = s + h/6*(k1 + 2*k2 + 2*k3 + k4);
        % batente no fim do trilho: choque inelástico do carrinho
        if abs(s(1)) > x_lim
            s(1) = sign(s(1))*x_lim;
            if s(2)*sign(s(1)) > 0
                v = s(2);
                s(4) = s(4) + p.m*p.l*cos(s(3))*v/(p.I + p.m*p.l^2);
                s(2) = 0;
            end
            dados.fim_curso(k) = true;
        end
    end
    dados.caiu(k) = abs(s(3)) > pi/2;
    if dados.caiu(k) && cfg.parar_ao_cair, k_fim = k; break; end

    % ---- interface ----
    if cfg.gui && (mod(k, cfg.passo_anim) == 0 || k == N)
        atualizar_gui(gui, dados, k, s, u, F_c, F_p, p, cfg);
    end

    % ---- tempo real ----
    if cfg.tempo_real
        espera = (k*Ts) - (toc(t_ref) - t_pausa);
        if espera > 0, pause(espera); end
    end
end

% ---------------- finalização ----------------
campos = {'t','X','Y','u_ctrl','u','F_carro','F_ponta','t_calc_us','t_troca_ms','fim_curso','caiu'};
for i = 1:numel(campos), dados.(campos{i}) = dados.(campos{i})(1:k_fim, :); end
dados.tempo_parede = toc(t_ref) - t_pausa;
if ~isempty(cfg.salvar)
    save(cfg.salvar, 'dados');
    fprintf('Dados salvos em %s\n', cfg.salvar);
end
if cfg.graficos_finais && k_fim > 1
    plotar_resultados(dados);
end
end

function cfg = config_padrao()
cfg.modo        = 'hil';        % 'sim' | 'hil'
cfg.porta       = 'COM3';           % vazio = auto; ex.: 'COM3', '/dev/ttyACM0'
cfg.baud        = 115200;       % igual ao BAUD do sketch
cfg.timeout     = 1.0;          % [s]
cfg.controlador = 'kan';        % 'lqr' | 'rede' | 'kan'
cfg.rede        = [];
cfg.kan         = [];
cfg.K           = [-20.0742 -13.9816 -52.9700 -10.0757];
cfg.Ts          = 0.01;
cfg.subpassos   = 10;
cfg.Tfinal      = 30;
cfg.x0 = [0 0 deg2rad(210) 0];
cfg.ruido_med   = [0 0 0 0];
cfg.perturbacao = [];
cfg.F_carro_max = 5.0;
cfg.F_ponta_max = 1.0;
cfg.dur_empurrao = 0.1;
cfg.janela_graf = 5.0;
cfg.passo_anim  = 1;
cfg.parar_ao_cair = false;
cfg.gui         = true;
cfg.tempo_real  = true;
cfg.graficos_finais = true;
cfg.salvar      = [];
cfg.planta      = [];
end

% =====================================================================
function gui = criar_gui(p, cfg, nome_ctrl)
gui.fig = figure('Name', ['Pêndulo invertido HIL - ' nome_ctrl], 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.08 0.9 0.82]);
setappdata(gui.fig, 'parar', false);
setappdata(gui.fig, 'pausado', false);
setappdata(gui.fig, 'F_carro', 0);
setappdata(gui.fig, 'F_ponta', cfg.F_ponta_max*0.4);
setappdata(gui.fig, 'pedido_empurrao', false);

% ---- animação ----
xl = p.L_trilho/2; Lh = p.Lh;
gui.ax_anim = axes('Parent', gui.fig, 'Position', [0.05 0.42 0.42 0.53]);
hold(gui.ax_anim, 'on'); axis(gui.ax_anim, 'equal');
lim_x = max(xl + Lh*0.7, xl + 0.15);
set(gui.ax_anim, 'XLim', [-lim_x lim_x], 'YLim', [-Lh-0.1 Lh+0.15]);
grid(gui.ax_anim, 'on'); xlabel(gui.ax_anim, 'x [m]');
plot(gui.ax_anim, [-xl xl], [0 0], 'k-', 'LineWidth', 3);                     % trilho
plot(gui.ax_anim, [-xl -xl], [-0.02 0.09], 'r-', 'LineWidth', 4);             % batentes
plot(gui.ax_anim, [ xl  xl], [-0.02 0.09], 'r-', 'LineWidth', 4);
plot(gui.ax_anim, [0 0], [-0.03 0.03], 'k-');                                  % meio
gui.wc = 0.10; gui.hc = 0.05;
gui.carro = patch('Parent', gui.ax_anim, 'XData', [-1 1 1 -1]*gui.wc/2, ...
    'YData', [0 0 1 1]*gui.hc, 'FaceColor', [0.2 0.45 0.8], 'EdgeColor', 'k');
gui.haste = plot(gui.ax_anim, [0 0], [gui.hc gui.hc+Lh], '-', 'Color', [0.85 0.33 0.1], 'LineWidth', 4);
gui.massa = plot(gui.ax_anim, 0, gui.hc+Lh, 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.85 0.33 0.1], 'MarkerEdgeColor', 'k');
gui.seta_c = plot(gui.ax_anim, nan, nan, '-', 'Color', [0 0.6 0], 'LineWidth', 3);
gui.ponta_c = plot(gui.ax_anim, nan, nan, '>', 'Color', [0 0.6 0], 'MarkerFaceColor', [0 0.6 0]);
gui.seta_p = plot(gui.ax_anim, nan, nan, '-', 'Color', [0.6 0 0.6], 'LineWidth', 3);
gui.ponta_p = plot(gui.ax_anim, nan, nan, '>', 'Color', [0.6 0 0.6], 'MarkerFaceColor', [0.6 0 0.6]);
gui.titulo = title(gui.ax_anim, 't = 0.00 s');

% ---- gráficos ao vivo ----
cores = lines(4);
rot = {'x [m]', '\theta [graus]', 'u [N]', 'perturbação [N]'};
for i = 1:4
    gui.ax(i) = axes('Parent', gui.fig, 'Position', [0.56 0.77-(i-1)*0.235 0.41 0.19]);
    hold(gui.ax(i), 'on'); grid(gui.ax(i), 'on'); ylabel(gui.ax(i), rot{i});
end
xlabel(gui.ax(4), 't [s]');
plot(gui.ax(1), [0 1e6], [xl xl], 'r--'); plot(gui.ax(1), [0 1e6], -[xl xl], 'r--');
plot(gui.ax(3), [0 1e6], [p.F_max p.F_max], 'r--'); plot(gui.ax(3), [0 1e6], -[p.F_max p.F_max], 'r--');
set(gui.ax(1), 'YLim', [-xl xl]*1.15);
set(gui.ax(3), 'YLim', [-p.F_max p.F_max]*1.15);
gui.l_x  = plot(gui.ax(1), nan, nan, 'Color', cores(1,:), 'LineWidth', 1.5);
gui.l_th = plot(gui.ax(2), nan, nan, 'Color', cores(2,:), 'LineWidth', 1.5);
gui.l_u  = plot(gui.ax(3), nan, nan, 'Color', cores(3,:), 'LineWidth', 1.2);
gui.l_fc = plot(gui.ax(4), nan, nan, 'Color', [0 0.6 0], 'LineWidth', 1.5);
gui.l_fp = plot(gui.ax(4), nan, nan, 'Color', [0.6 0 0.6], 'LineWidth', 1.5);
legend(gui.ax(4), {'no carrinho', 'na ponta da haste'}, 'Location', 'northwest');

% ---- painel de perturbação ----
f = gui.fig; Fcm = cfg.F_carro_max; Fpm = cfg.F_ponta_max;
uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.05 0.31 0.42 0.03], ...
    'String', 'Força contínua no carrinho [N]', 'BackgroundColor', 'w', 'FontWeight', 'bold');
gui.sl_c = uicontrol(f, 'Style', 'slider', 'Units', 'normalized', 'Position', [0.05 0.27 0.34 0.035], ...
    'Min', -Fcm, 'Max', Fcm, 'Value', 0, 'SliderStep', [0.025 0.1], ...
    'Callback', @(src, ~) setappdata(f, 'F_carro', get(src, 'Value')));
gui.tx_c = uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.40 0.27 0.07 0.035], ...
    'String', '0.00 N', 'BackgroundColor', 'w');

uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.05 0.21 0.42 0.03], ...
    'String', sprintf('Empurrão na ponta da haste [N] (dura %.2f s)', cfg.dur_empurrao), ...
    'BackgroundColor', 'w', 'FontWeight', 'bold');
gui.sl_p = uicontrol(f, 'Style', 'slider', 'Units', 'normalized', 'Position', [0.05 0.17 0.26 0.035], ...
    'Min', -Fpm, 'Max', Fpm, 'Value', Fpm*0.4, 'SliderStep', [0.025 0.1], ...
    'Callback', @(src, ~) setappdata(f, 'F_ponta', get(src, 'Value')));
gui.tx_p = uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.32 0.17 0.06 0.035], ...
    'String', sprintf('%.2f N', Fpm*0.4), 'BackgroundColor', 'w');
uicontrol(f, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.39 0.165 0.08 0.045], ...
    'String', 'Empurrar', 'FontWeight', 'bold', ...
    'Callback', @(~, ~) setappdata(f, 'pedido_empurrao', true));

uicontrol(f, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.05 0.09 0.13 0.05], ...
    'String', 'Zerar perturbação', 'Callback', @(~, ~) zerar_pert(f, gui.sl_c));
gui.bt_pausa = uicontrol(f, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.195 0.09 0.13 0.05], ...
    'String', 'Pausar', 'Callback', @(src, ~) alternar_pausa(f, src));
uicontrol(f, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.34 0.09 0.13 0.05], ...
    'String', 'Parar', 'ForegroundColor', [0.7 0 0], 'FontWeight', 'bold', ...
    'Callback', @(~, ~) setappdata(f, 'parar', true));

gui.status = uicontrol(f, 'Style', 'text', 'Units', 'normalized', 'Position', [0.05 0.01 0.42 0.06], ...
    'String', ['Controlador: ' nome_ctrl], 'BackgroundColor', [0.95 0.95 0.95], ...
    'HorizontalAlignment', 'left');
drawnow;
end

function zerar_pert(f, sl)
setappdata(f, 'F_carro', 0); set(sl, 'Value', 0);
end

function alternar_pausa(f, bt)
pausado = ~getappdata(f, 'pausado');
setappdata(f, 'pausado', pausado);
if pausado, set(bt, 'String', 'Continuar'); else, set(bt, 'String', 'Pausar'); end
end

% =====================================================================
function atualizar_gui(gui, dados, k, s, u, F_c, F_p, p, cfg)
if ~ishandle(gui.fig), return; end
x = s(1); th = s(3); Lh = p.Lh; hc = gui.hc;
px = x + Lh*sin(th); py = hc + Lh*cos(th);
set(gui.carro, 'XData', x + [-1 1 1 -1]*gui.wc/2);
set(gui.haste, 'XData', [x px], 'YData', [hc py]);
set(gui.massa, 'XData', px, 'YData', py);
esc_c = 0.03; esc_p = 0.15;  % m por N no desenho das setas
desenhar_seta(gui.seta_c, gui.ponta_c, x - sign(F_c)*(gui.wc/2 + esc_c*abs(F_c)), hc/2, esc_c*F_c);
desenhar_seta(gui.seta_p, gui.ponta_p, px - sign(F_p)*(0.03 + esc_p*abs(F_p)), py, esc_p*F_p);

t = dados.t(k);
msg = sprintf('t = %.2f s   u = %+.2f N', t, u);
cor = 'k';
if any(dados.fim_curso(max(1, k-cfg.passo_anim+1):k)), msg = [msg '   FIM DE CURSO!']; cor = 'r'; end
if dados.caiu(k), msg = [msg '   PÊNDULO CAIU']; cor = 'r'; end
set(gui.titulo, 'String', msg, 'Color', cor);

i0 = max(1, k - round(cfg.janela_graf/cfg.Ts));
tt = dados.t(i0:k);
set(gui.l_x,  'XData', tt, 'YData', dados.X(i0:k, 1));
set(gui.l_th, 'XData', tt, 'YData', dados.X(i0:k, 3)*180/pi);
set(gui.l_u,  'XData', tt, 'YData', dados.u(i0:k));
set(gui.l_fc, 'XData', tt, 'YData', dados.F_carro(i0:k));
set(gui.l_fp, 'XData', tt, 'YData', dados.F_ponta(i0:k));
xlim_v = [max(0, t - cfg.janela_graf), max(t, cfg.janela_graf)];
set(gui.ax, 'XLim', xlim_v);

set(gui.tx_c, 'String', sprintf('%.2f N', getappdata(gui.fig, 'F_carro')));
set(gui.tx_p, 'String', sprintf('%.2f N', getappdata(gui.fig, 'F_ponta')));
st = sprintf('Controlador: %s', dados.nome_ctrl);
if ~isnan(dados.t_troca_ms(k))
    st = sprintf('%s\nida e volta serial: %.1f ms | cálculo no Arduino: %.0f us | Ts = %.0f ms', ...
        st, dados.t_troca_ms(k), dados.t_calc_us(k), 1e3*cfg.Ts);
end
set(gui.status, 'String', st);
try
    drawnow limitrate
catch
    drawnow
end
end

function desenhar_seta(linha, ponta, x0, y0, dx)
if abs(dx) < 1e-6
    set(linha, 'XData', nan, 'YData', nan); set(ponta, 'XData', nan, 'YData', nan); return;
end
set(linha, 'XData', [x0 x0+dx], 'YData', [y0 y0]);
if dx > 0, mk = '>'; else, mk = '<'; end
set(ponta, 'XData', x0+dx, 'YData', y0, 'Marker', mk);
end

% =====================================================================
function [sp, porta] = hil_abrir(cfg)
porta = cfg.porta;
if isempty(porta)
    if exist('serialportlist', 'file') == 2
        lst = serialportlist('available');
    else
        try
            info = instrhwinfo('serial');
            lst = info.AvailableSerialPorts;
        catch
            error('Nenhuma porta serial disponivel. Conecte o Arduino ou informe ''porta''.');
        end
    end
    if isempty(lst)
        error('Nenhuma porta serial disponivel. Conecte o Arduino ou informe ''porta''.');
    end
    porta = char(lst(end));
    fprintf('Usando a porta %s\n', porta);
end

if exist('serialport', 'file') == 2
    sp = serialport(porta, cfg.baud, 'Timeout', cfg.timeout);
    configureTerminator(sp, 'LF');
else
    % Compatibilidade com MATLAB mais antigo: usa objeto serial
    sp = serial(porta, 'BaudRate', cfg.baud);
    set(sp, 'Timeout', cfg.timeout);
    fopen(sp);
    set(sp, 'Terminator', 'LF');
end

fprintf('Aguardando o Arduino reiniciar...\n');
pause(2.0);                               % a placa reinicia ao abrir a porta
if exist('serialport', 'file') == 2
    flush(sp);
else
    flushinput(sp); flushoutput(sp);
end
ok = false;
for tent = 1:6
    if exist('serialport', 'file') == 2
        writeline(sp, '?');
        l = readline(sp);
    else
        fprintf(sp, '?\n');
        l = fgetl(sp);
    end
    if ~isempty(l) && strncmp(strtrim(char(l)), 'PRONTO', 6), ok = true; break; end
    pause(0.5);
    if exist('serialport', 'file') == 2
        flush(sp);
    else
        flushinput(sp); flushoutput(sp);
    end
end
if ~ok
    error(['O Arduino nao respondeu ao "?". Confira a porta, o baud (%d) e se o sketch ' ...
           'controlador_pendulo.ino foi carregado.'], cfg.baud);
end
if exist('serialport', 'file') == 2
    writeline(sp, 'R'); readline(sp);         % zera estados internos do controlador
else
    fprintf(sp, 'R\n'); fgetl(sp);
end
fprintf('Arduino pronto em %s.\n', porta);
porta = char(porta);
end

function [u, t_us] = hil_trocar(sp, y)
msg = sprintf('%.6f,%.6f,%.6f,%.6f', y);
if exist('serialport', 'file') == 2
    writeline(sp, msg);
    l = readline(sp);
else
    fprintf(sp, [msg '\n']);
    l = fgetl(sp);
end
if isempty(l)
    error('O Arduino não respondeu dentro do timeout. Aumente ''timeout'' ou ''Ts''.');
end
v = sscanf(char(l), '%f,%f');
if isempty(v)
    error('Resposta inválida do Arduino: "%s"', char(l));
end
u = v(1);
t_us = NaN; if numel(v) >= 2, t_us = v(2); end
end

function hil_fechar(sp)
if isempty(sp), return; end

try
    if exist('serialport', 'file') == 2
        try
            writeline(sp, 'R');     % deixa o controlador zerado para a próxima execução
            pause(0.05);
        catch
        end
        try
            flush(sp);
            pause(0.05);
        catch
        end
        try
            delete(sp);
        catch
        end
    else
        try
            fprintf(sp, 'R\n');
            pause(0.05);
            flushinput(sp); flushoutput(sp);
        catch
        end
        try
            fclose(sp);
        catch
        end
        try
            delete(sp);
        catch
        end
    end
catch
end
end
