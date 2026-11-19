function Y = ident_kan_forward(kan, X)
%IDENT_KAN_FORWARD  Avalia a KAN (formato pykan) para várias amostras de uma vez.
%   Y = ident_kan_forward(kan, X)     X (N x n_ent) JÁ normalizado, Y (N x n_sai)
%   Para cada camada e cada saída j:
%     s_j = sum_i sb_ij*SiLU(x_i) + sp_ij*sum_m coef_ijm*B_m(x_i)
%     x_j = a_j*s_j + c_j
%   É a mesma conta de kan_eval.h no ESP32 (lá em float).

for l = 1:numel(kan.camadas)
    C = kan.camadas{l};
    [ne, ns] = size(C.sb);
    N = size(X, 1);
    S = zeros(N, ns);
    for i = 1:ne
        xi = X(:, i);
        Bi = ident_kan_bsplines(xi, C.grid(i, :), kan.k);
        Ci = reshape(C.coef(i, :, :), ns, []);             % ns x nb
        silu = xi./(1 + exp(-xi));
        S = S + silu*C.sb(i, :) + bsxfun(@times, Bi*Ci.', C.sp(i, :));
    end
    X = bsxfun(@plus, bsxfun(@times, S, C.a(:).'), C.c(:).');
end
Y = X;
end
