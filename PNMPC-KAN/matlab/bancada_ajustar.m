function p = bancada_ajustar(arquivo, mostrar)
%BANCADA_AJUSTAR  Ajusta w0q, kl, c do simulador bancada_planta a um ensaio da bancada.
%   p = bancada_ajustar(arquivo)      (padrão: ensaio_20261119_154800.csv)
%
%   Erro de simulação por segmento: cada segmento do ensaio é simulado a partir do
%   estado medido no seu início, com a a_cmd gravada, e compara-se theta simulado
%   com o medido. Minimiza a soma (fminsearch), partindo do ajuste por mínimos quadrados.
%   Salva em bancada_param.mat.

aqui = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(arquivo)
    d = dir(fullfile(aqui, '..', 'Planta_smilua*'));
    arquivo = fullfile(aqui, '..', d(1).name, 'ensaio_20261119_154800.csv');
end
if nargin < 2, mostrar = true; end
D = ident_ler_bancada(arquivo);
sr = D.serie;
segs = unique(sr.seg(sr.seg >= 1));
segs = segs(arrayfun(@(g) sum(sr.seg == g), segs) > 30);   % segmentos curtos não ajudam

custo = @(q) erro_total(q, sr, segs);
q0 = [43.7 4.24 0.34];
opc = optimset('Display', 'off', 'MaxFunEvals', 400, 'TolX', 1e-4, 'TolFun', 1e-8);
q = fminsearch(custo, q0, opc);
p = struct('w0q', q(1), 'kl', q(2), 'c', q(3));
[e, ~] = erro_total(q, sr, segs);
[e0, ~] = erro_total(q0, sr, segs);
if mostrar
    fprintf('Planta da bancada ajustada: w0q = %.3f (f0 = %.3f Hz), kl = %.3f (l = %.3f m), c = %.3f\n', ...
        p.w0q, sqrt(p.w0q)/2/pi, p.kl, 1/p.kl, p.c);
    fprintf('  RMSE de theta simulado (por segmento): %.3f graus (inicial %.3f)\n', ...
        sqrt(e)*180/pi, sqrt(e0)*180/pi);
end
p.arquivo = arquivo;
save(fullfile(aqui, 'bancada_param.mat'), 'p');
end

function [e, n] = erro_total(q, sr, segs)
Ts = 0.01;
e = 0; n = 0;
for g = segs(:).'
    r = find(sr.seg == g);
    th = sr.S(r(1), 3); w = sr.S(r(1), 4);
    for i = 1:numel(r) - 1
        a = sr.U(r(i));
        for j = 1:4                               % RK4 com 4 subpassos
            h = Ts/4;
            k1 = ff(th, w, a, q);
            k2 = ff(th + h/2*k1(1), w + h/2*k1(2), a, q);
            k3 = ff(th + h/2*k2(1), w + h/2*k2(2), a, q);
            k4 = ff(th + h*k3(1), w + h*k3(2), a, q);
            th = th + h/6*(k1(1) + 2*k2(1) + 2*k3(1) + k4(1));
            w  = w  + h/6*(k1(2) + 2*k2(2) + 2*k3(2) + k4(2));
        end
        e = e + (th - sr.S(r(i+1), 3))^2; n = n + 1;
    end
end
e = e/n;
end

function d = ff(th, w, a, q)
d = [w, -q(1)*sin(th) - q(2)*a*cos(th) - q(3)*w];
end
