classdef bancada_planta < handle
%BANCADA_PLANTA  Simulador da bancada (carrinho com motor de passo + pêndulo) para o HIL.
%
%   pl = bancada_planta(p)            p = bancada_ajustar(...) (ou [] = valores padrão)
%   pl.reiniciar([x v th w])          estado inicial (th = 0: pendurado)
%   y = pl.passo(a, Ts)               aplica a aceleração a [m/s^2] por Ts (ZOH) e devolve
%                                     a MEDIÇÃO [x v th w] como o firmware pid_IA faz
%
%   Dinâmica (carrinho imposto pelo motor de passo):
%     x'  = v,   v' = a
%     th'' = -w0q*sin(th) - kl*a*cos(th) - c*th'
%   Medição (igual ao firmware): encoder de 2400 contagens/volta (0,15 graus) lido a cada
%   0,5 ms; omega = 0.5*omega + 0.5*(th - th_8_amostras_atras)/janela (janela de 4 ms).

    properties
        p                % parâmetros: w0q, kl, c, dt_med, cont_volta, v_max, x_lim
        s                % estado verdadeiro [x v th w]
        th_hist          % histórico do ângulo medido (8 amostras)
        idx = 1
        w_med = 0        % omega medido (filtrado)
        th_med = 0
    end

    methods
        function obj = bancada_planta(p)
            pad = struct('w0q', 43.7, 'kl', 4.24, 'c', 0.34, 'dt_med', 5e-4, ...
                'cont_volta', 2400, 'quantizar', true, 'x_lim', 0.20);
            if nargin < 1 || isempty(p), p = struct(); end
            nomes = fieldnames(pad);
            for i = 1:numel(nomes)
                if ~isfield(p, nomes{i}), p.(nomes{i}) = pad.(nomes{i}); end
            end
            obj.p = p;
            obj.reiniciar([0 0 0 0]);
        end

        function reiniciar(obj, s0)
            obj.s = s0(:).';
            obj.th_med = obj.medir_th(obj.s(3));
            obj.th_hist = obj.th_med*ones(1, 8);
            obj.idx = 1;
            obj.w_med = 0;
        end

        function y = passo(obj, a, Ts)
            n = max(1, round(Ts/obj.p.dt_med));
            h = Ts/n;
            for i = 1:n
                obj.s = rk4(obj.s, a, h, obj.p);
                % estimador do firmware (atualizarAngulo), a cada dt_med
                obj.th_med = obj.medir_th(obj.s(3));
                bruto = (obj.th_med - obj.th_hist(obj.idx))/(8*h);
                obj.w_med = 0.5*obj.w_med + 0.5*bruto;
                obj.th_hist(obj.idx) = obj.th_med;
                obj.idx = mod(obj.idx, 8) + 1;
            end
            y = [obj.s(1), obj.s(2), obj.th_med, obj.w_med];
        end

        function th = medir_th(obj, th)
            if obj.p.quantizar
                q = 2*pi/obj.p.cont_volta;
                th = q*round(th/q);
            end
        end
    end
end

function s = rk4(s, a, h, p)
k1 = f(s, a, p); k2 = f(s + h/2*k1, a, p); k3 = f(s + h/2*k2, a, p); k4 = f(s + h*k3, a, p);
s = s + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function ds = f(s, a, p)
ds = [s(2), a, s(4), -p.w0q*sin(s(3)) - p.kl*a*cos(s(3)) - p.c*s(4)];
end
