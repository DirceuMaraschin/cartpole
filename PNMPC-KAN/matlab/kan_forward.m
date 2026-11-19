function u = kan_forward(kan, y)
%KAN_FORWARD  Avalia no MATLAB a mesma KAN (pykan) que roda no Arduino.
%   u = kan_forward(kan, [x; dx; th; dth])      kan = kan_pesos()
%   Para cada camada e saída j:
%     s_j = sum_i sb_ij*SiLU(x_i) + sp_ij*sum_m coef_ijm*B_m(x_i)
%     x_j = a_j*s_j + c_j

x = kan.ent_ganho(:).*y(:) + kan.ent_desloc(:);
k = kan.k;
for l = 1:numel(kan.camadas)
    C = kan.camadas{l};
    [n_ent, n_sai] = size(C.sb);
    s = zeros(n_sai, 1);
    for i = 1:n_ent
        B = bsplines(x(i), C.grid(i, :), k);              % (nb x 1)
        silu = x(i)/(1 + exp(-x(i)));
        s = s + C.sb(i, :).'*silu + C.sp(i, :).'.*(reshape(C.coef(i, :, :), n_sai, []) * B);
    end
    x = C.a(:).*s + C.c(:);
end
u = kan.sai_ganho*x(1) + kan.sai_desloc;
end

function B = bsplines(x, g, k)
% Cox-de Boor, igual a kan.spline.B_batch (grid já estendido)
g = g(:);
B = double(x >= g(1:end-1) & x < g(2:end));
for p = 1:k
    n = numel(g) - 1 - p;
    Bn = zeros(n, 1);
    for m = 1:n
        d1 = g(m+p) - g(m);
        d2 = g(m+p+1) - g(m+1);
        t1 = 0; t2 = 0;
        if d1 ~= 0, t1 = (x - g(m))/d1*B(m); end
        if d2 ~= 0, t2 = (g(m+p+1) - x)/d2*B(m+1); end
        Bn(m) = t1 + t2;
    end
    B = Bn;
end
end
