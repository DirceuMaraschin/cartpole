function [u, est, info] = pnmpc_kan_passo(kan, mpc, est, s, r_x, th_ref)
%PNMPC_KAN_PASSO  Um passo do PNMPC iterativo com a KAN (mesma conta de pnmpc_kan.h no ESP32).
%   [u, est, info] = pnmpc_kan_passo(kan, mpc, est, s, r_x, th_ref)
%   kan    KAN (kan_volta.mat)             mpc = pnmpc_param()
%   est    estado interno ([] na 1a chamada)
%   s      medição [x v th w] (th = 0 pendurado, pi = em pé), r_x [m], th_ref [rad]
%
%   Saídas do custo: o = [x, v, wrap(th - th_ref), w, E, max(0,|x| - x_soft)],
%   E = 0.5*w^2/w0q - cos(th). Pesos de SWING-UP (energia) ou de EQUILÍBRIO (ângulo),
%   escolhidos com histerese pelo erro de ângulo medido (phi_sw).
%   1) inovação n(k) = y(k) - y_n(k) (filtrada, em x e theta)
%   2) mpc.iters vezes: resposta livre, G por perturbação, QP com limites, u_base += du
%   3) u = u_base(1)

NU = numel(mpc.bloco); N = mpc.N; NO = 7;
if isempty(est)
    est = struct('ub', zeros(1, NU), 'u_ant', 0, 'nf', zeros(1, 4), 'pred', zeros(1, 4), ...
        'tem_pred', false, 'equilibrio', false);
end
s = s(:).';

% ---- modo ----
eth = abs(wrapp(s(3) - th_ref));
if est.equilibrio
    if eth > 1.5*mpc.phi_sw, est.equilibrio = false; end
elseif eth < mpc.phi_sw
    est.equilibrio = true;
end
q_vs = 0; if isfield(mpc, 'q_vs'), q_vs = mpc.q_vs; end
if est.equilibrio, Wc = [mpc.q 0 mpc.q_xs q_vs]; else, Wc = [mpc.q_sw_x mpc.q_sw_v 0 0 mpc.q_E mpc.q_xs q_vs]; end
ref = [r_x 0 0 0 mpc.E_alvo 0 0];

% ---- 1) inovação n(k) = y(k) - y_n(k) ----
paralelo = isfield(mpc, 'paralelo') && mpc.paralelo;
s0 = s;                                    % de onde parte a resposta livre
if paralelo
    % como na figura: modelo PARALELO (só recebe u); y_n(k) = estado do modelo
    if ~isfield(est, 'sm') || isempty(est.sm), est.sm = s; end
    d = s - est.sm; d(3) = wrapp(d(3));
    est.nf = mpc.alfa_n*est.nf + (1 - mpc.alfa_n)*d;
    n = [est.nf 0 0 0];
    s0 = est.sm;
else
    if est.tem_pred
        est.nf(1) = mpc.alfa_n*est.nf(1) + (1 - mpc.alfa_n)*(s(1) - est.pred(1));
        est.nf(3) = mpc.alfa_n*est.nf(3) + (1 - mpc.alfa_n)*wrapp(s(3) - est.pred(3));
    end
    n = [est.nf(1) 0 est.nf(3) 0 0 0 0];
end

ub = min(max(est.ub, mpc.u_min), mpc.u_max);
idx = ones(1, N);
for b = 2:NU, idx((mpc.bloco(b) + 1):end) = b; end
mult = ones(N, 1);
if ~est.equilibrio, mult(end) = mpc.q_term; end   % no equilíbrio o terminal é o LQR
W = bsxfun(@times, mult, Wc);                       % N x NO

% ---- 2) laço iterativo ----
for it = 1:mpc.iters
    Useq = [ub; repmat(ub, NU, 1) + mpc.eps*eye(NU)];
    X = rollout(kan, s0, Useq, idx);                % (1+NU) x N x 4
    Ob = saidas(squeeze(X(1, :, :)), th_ref, mpc);  % N x NO
    G = zeros(N, NO, NU);
    for j = 1:NU
        d = saidas(squeeze(X(1 + j, :, :)), th_ref, mpc) - Ob;
        d(:, 3) = wrapp(d(:, 3));
        G(:, :, j) = d/mpc.eps;
        G(1:mpc.bloco(j), :, j) = 0;
    end
    E = bsxfun(@minus, ref, bsxfun(@plus, Ob, n));
    E(:, 3) = wrapp(E(:, 3));
    H = zeros(NU); f = zeros(NU, 1);
    for j = 1:NU
        f(j) = -sum(sum(W.*G(:, :, j).*E));
        for l = 1:NU, H(j, l) = sum(sum(W.*G(:, :, j).*G(:, :, l))); end
    end
    cT = 0;
    if est.equilibrio                       % custo terminal LQR: e^T P e no último passo
        Pm = mpc.Pt([1 2 3 4; 2 5 6 7; 3 6 8 9; 4 7 9 10]);
        eT = E(N, 1:4).';
        GT = reshape(G(N, 1:4, :), 4, NU);
        cT = eT.'*Pm*eT;
        f = f - GT.'*Pm*eT;
        H = H + GT.'*Pm*GT;
    end
    dd = ub - [est.u_ant ub(1:end-1)];
    for j = 1:NU
        if j < NU, H(j, j) = H(j, j) + 2*mpc.r_du; dn = dd(j+1); else, H(j, j) = H(j, j) + mpc.r_du; dn = 0; end
        H(j, j) = H(j, j) + mpc.r_u;
        if j > 1, H(j, j-1) = H(j, j-1) - mpc.r_du; H(j-1, j) = H(j-1, j) - mpc.r_du; end
        f(j) = f(j) + mpc.r_du*(dd(j) - dn) + mpc.r_u*ub(j);
    end
    custo = sum(sum(W.*E.^2)) + cT + mpc.r_du*sum(dd.^2) + mpc.r_u*sum(ub.^2);
    du = zeros(NU, 1);
    for v = 1:mpc.varreduras
        for j = 1:NU
            if H(j, j) <= 0, continue; end
            gr = f(j) + H(j, :)*du;
            du(j) = min(max(du(j) - gr/H(j, j), mpc.u_min - ub(j)), mpc.u_max - ub(j));
        end
    end
    ub = ub + du.';
end

% ---- 3) aplica ----
u = ub(1);
est.ub = ub; est.u_ant = u;
if paralelo
    est.sm = ident_kan_prever(kan, est.sm, u);     % avança o modelo paralelo
else
    est.pred = ident_kan_prever(kan, s, u);        % y_n(k+1)
    est.tem_pred = true;
end
info = struct('u', u, 'custo', custo, 'n_x', n(1), 'n_th', n(3), 'modo', double(est.equilibrio), ...
    'E', 0.5*s(4)^2/mpc.w0q - cos(s(3)));
end

function O = saidas(X, th_ref, mpc)
O = [X(:, 1), X(:, 2), wrapp(X(:, 3) - th_ref), X(:, 4), ...
     0.5*X(:, 4).^2/mpc.w0q - cos(X(:, 3)), max(0, abs(X(:, 1)) - mpc.x_soft), vexc(X, mpc)];
end

function X = rollout(kan, s0, Useq, idx)
ns = size(Useq, 1); N = numel(idx);
X = zeros(ns, N, 4);
S = repmat(s0, ns, 1);
for i = 1:N
    S = ident_kan_prever(kan, S, Useq(:, idx(i)));
    X(:, i, :) = reshape(S, ns, 1, 4);
end
end

function a = wrapp(a)
a = a - 2*pi*floor((a + pi)/(2*pi));
end

function e = vexc(X, mpc)
% penalidade suave de velocidade (mesma de pnmpc_kan.h)
if isfield(mpc, 'v_soft') && mpc.v_soft > 0, e = max(0, abs(X(:, 2)) - mpc.v_soft); else, e = zeros(size(X, 1), 1); end
end
