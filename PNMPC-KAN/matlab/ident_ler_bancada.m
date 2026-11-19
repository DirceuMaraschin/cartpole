function D = ident_ler_bancada(arquivo, cfg)
%IDENT_LER_BANCADA  Lê um ensaio da bancada (CSV/XLSX do firmware pid_IA) como dados de planta.
%   D = ident_ler_bancada('..\Planta_smiluação\ensaio_20261119_154800.csv')
%   D = ident_ler_bancada(arquivo, struct('segmentos', 1:14))
%
%   Colunas esperadas: t_s; seg; alvo_mm; x_mm; v_cmd_ms; v_med_ms; a_cmd_ms2; theta_deg; omega_rads
%   (separador ';'). O carrinho é um motor de passo: a ENTRADA da planta é a
%   aceleração comandada a_cmd [m/s^2] e o estado é
%       s = [x (m), v_med (m/s), theta (rad, 0 = pêndulo pendurado), omega (rad/s)]
%
%   Devolve no mesmo formato de ident_gerar_dados:
%     D.S (N x 4) estado em k, D.U (N x 1) a_cmd em k, D.S1 (N x 4) estado em k+1,
%     D.ep (N x 1) segmento do ensaio, e a série completa em D.serie (t, S, U, seg).
%   Só entram pares (k, k+1) com intervalo entre 0,8*Ts e 1,2*Ts.
%
%   cfg.segmentos: segmentos usados (padrão: todos com seg >= 0).
%   cfg.Ts: período nominal (padrão 0.01 s).

if nargin < 2, cfg = struct(); end
if ~isfield(cfg, 'Ts'), cfg.Ts = 0.01; end
if ~isfield(cfg, 'segmentos'), cfg.segmentos = []; end

[~, ~, ext] = fileparts(arquivo);
if any(strcmpi(ext, {'.xlsx', '.xls'}))
    M = xlsread(arquivo);
    M = M(all(~isnan(M(:, 1:9)), 2), 1:9);
else
    f = fopen(arquivo, 'r');
    if f < 0, error('Não consegui abrir %s', arquivo); end
    fgetl(f);                                          % cabeçalho
    C = textscan(f, '%f%f%f%f%f%f%f%f%f', 'Delimiter', ';');
    fclose(f);
    M = [C{:}];
end
t   = M(:, 1); seg = M(:, 2);
x   = M(:, 4)/1000;                % mm -> m
v   = M(:, 6);                     % velocidade medida [m/s]
a   = M(:, 7);                     % aceleração comandada [m/s^2] (entrada)
th  = M(:, 8)*pi/180;              % graus -> rad, 0 = pendurado
om  = M(:, 9);                     % rad/s

S = [x v th om];
if isempty(cfg.segmentos), usar = seg >= 0; else, usar = ismember(seg, cfg.segmentos); end
dt = diff(t);
ok = usar(1:end-1) & usar(2:end) & dt > 0.8*cfg.Ts & dt < 1.2*cfg.Ts;
k = find(ok);

D.S = S(k, :); D.U = a(k); D.S1 = S(k+1, :); D.ep = seg(k);
D.cfg = cfg; D.cfg.arquivo = arquivo;
D.serie = struct('t', t, 'S', S, 'U', a, 'seg', seg, 'alvo', M(:, 3)/1000, 'v_cmd', M(:, 5));
fprintf('Bancada: %s\n  %d amostras, %d pares (k,k+1) usados, segmentos %s\n', arquivo, ...
    numel(t), numel(k), mat2str(unique(D.ep)'));
fprintf('  faixas: |x| <= %.3f m, |v| <= %.3f m/s, |theta| <= %.1f graus, |omega| <= %.2f rad/s, |a_cmd| <= %.2f m/s^2\n', ...
    max(abs(D.S(:, 1))), max(abs(D.S(:, 2))), max(abs(D.S(:, 3)))*180/pi, max(abs(D.S(:, 4))), max(abs(D.U)));
end
