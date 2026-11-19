function S1 = ident_kan_prever(kan, S, U)
%IDENT_KAN_PREVER  Previsão um passo à frente da planta pela KAN.
%   S1 = ident_kan_prever(kan, S, U)
%   S  (N x 4) estados [x dx th dth] no instante k, U (N x 1) força aplicada em [k, k+1)
%   S1 (N x 4) estados previstos no instante k+1.
%
%   Regressores (iguais aos do ESP32):
%     [dx, phi, dth, u],  phi = wrap(th - th_eq)   (th_eq = pi: pêndulo pendurado)
%   A KAN devolve o incremento normalizado do estado:
%     dS = sai_ganho .* kan(xn) + sai_desloc ;  S1 = S + dS
%   x não entra na KAN porque a dinâmica não depende da posição do carrinho.

%   Se kan.lin (5 x 4) existir, soma a parte linear: dS += [v phi w u 1]*lin.

X  = ident_regressores(kan, S, U);
Yn = ident_kan_forward(kan, X);
dS = bsxfun(@plus, bsxfun(@times, Yn, kan.sai_ganho(:).'), kan.sai_desloc(:).');
if isfield(kan, 'lin') && any(kan.lin(:))
    F = bsxfun(@rdivide, bsxfun(@minus, X, kan.ent_desloc(:).'), kan.ent_ganho(:).');
    dS = dS + [F, ones(size(F, 1), 1)]*kan.lin;
end
S1 = S + dS;
end
