function [B, dB] = ident_kan_bsplines(x, g, k)
%IDENT_KAN_BSPLINES  B-splines de ordem k (Cox-de Boor, igual ao pykan) e derivadas.
%   [B, dB] = ident_kan_bsplines(x, g, k)
%   x  (N x 1) pontos, g (1 x ng) grid já estendido, B e dB (N x (ng-k-1)).
%   Intervalos semiabertos [g_m, g_m+1): fora do grid todas as bases valem 0.
%   (Compatível com R2016a: usa bsxfun, sem expansão implícita.)

x = x(:); g = g(:).';
B = double(bsxfun(@ge, x, g(1:end-1)) & bsxfun(@lt, x, g(2:end)));
Bkm1 = B;
for p = 1:k
    if p == k, Bkm1 = B; end
    n = numel(g) - 1 - p;
    g0 = g(1:n); gp = g(1+p:n+p); g1 = g(2:n+1); gp1 = g(2+p:n+p+1);
    t1 = bsxfun(@rdivide, bsxfun(@minus, x, g0), gp - g0) .* B(:, 1:n);
    t2 = bsxfun(@rdivide, bsxfun(@minus, gp1, x), gp1 - g1) .* B(:, 2:n+1);
    B = t1 + t2;
end
if nargout > 1
    n = size(B, 2);
    gk0 = g(1:n); gk = g(1+k:n+k); g1 = g(2:n+1); gk1 = g(2+k:n+k+1);
    dB = k*(bsxfun(@rdivide, Bkm1(:, 1:n), gk - gk0) - bsxfun(@rdivide, Bkm1(:, 2:n+1), gk1 - g1));
end
end
