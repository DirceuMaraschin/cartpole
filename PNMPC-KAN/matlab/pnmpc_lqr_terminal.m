function [Pt, P, K, A, B] = pnmpc_lqr_terminal(kan, mpc)
%PNMPC_LQR_TERMINAL  Custo terminal do PNMPC-KAN no equilíbrio: P da Riccati (DARE).
%   [Pt, P, K, A, B] = pnmpc_lqr_terminal(kan, mpc)
%   A e B vêm da PRÓPRIA KAN, linearizada por diferenças centrais em torno do pêndulo
%   em pé (s = [0 0 pi 0], a = 0). Q = diag(mpc.q) (pesos por passo do horizonte) e
%   R = mpc.r_lqr. Pt = triangular superior de P (10 valores, ordem do firmware:
%   p11 p12 p13 p14 p22 p23 p24 p33 p34 p44). K é o ganho LQR (só informativo).

s0 = [0 0 pi 0]; u0 = 0; d = 1e-4;
f = @(s, u) ident_kan_prever(kan, s, u);
A = zeros(4); B = zeros(4, 1);
for i = 1:4
    e = zeros(1, 4); e(i) = d;
    A(:, i) = (f(s0 + e, u0) - f(s0 - e, u0)).'/(2*d);
end
B(:, 1) = (f(s0, u0 + d) - f(s0, u0 - d)).'/(2*d);

Q = diag(mpc.q); R = mpc.r_lqr;
P = Q;
for it = 1:20000
    Pn = Q + A.'*P*A - (A.'*P*B)*((R + B.'*P*B) \ (B.'*P*A));
    if max(abs(Pn(:) - P(:))) < 1e-9*max(abs(Pn(:))), P = Pn; break; end
    P = Pn;
end
P = (P + P.')/2;
K = (R + B.'*P*B) \ (B.'*P*A);
Pt = [P(1,1) P(1,2) P(1,3) P(1,4) P(2,2) P(2,3) P(2,4) P(3,3) P(3,4) P(4,4)];
end
