function mpc = pnmpc_param(kan)
%PNMPC_PARAM  Sintonia do PNMPC-KAN (enviada ao ESP32 pelo comando P, 31 valores).
%   mpc = pnmpc_param          (usa kan_volta.mat para o custo terminal LQR)
%   mpc = pnmpc_param(kan)
%   Se mudar mpc.q, mpc.r_u ou mpc.r_du, recalcule: mpc.Pt = pnmpc_lqr_terminal(kan, mpc).
%   N e bloco são fixos no firmware (MPC_N, MPC_BLOCO em pnmpc_kan.h): mude nos dois.
%   Ângulos internos em rad na convenção da bancada (0 = pendurado, pi = em pé).

mpc.N        = 40;               % horizonte [passos de 10 ms] = 0,4 s
mpc.bloco    = [0 5 10 20];      % passo em que cada movimento de controle começa
% --- EQUILÍBRIO (perto da referência de ângulo) ---
mpc.q        = [300 20 120 0.5]; % pesos do erro de [x v theta omega] (SI)
mpc.q_term   = 3;                % multiplicador no último passo
% --- SWING-UP (longe da referência) ---
mpc.q_E      = 100;              % peso do erro de energia E = 0.5*w^2/w0q - cos(theta)
mpc.E_alvo   = 1.1;              % energia alvo (1 = em pé parado; um pouco acima compensa perdas)
mpc.w0q      = 43.67;            % omega0^2 do pendulo (ajustado ao log real da bancada)
mpc.q_sw_x   = 300;              % peso de x durante o swing-up
mpc.q_sw_v   = 50;               % peso de v durante o swing-up
mpc.phi_sw   = 30*pi/180;        % |erro de ângulo| para entrar no equilíbrio (sai com 1,5x)
% --- trilho e entrada ---
mpc.q_xs     = 2000;             % penalidade de |x| acima de x_soft
mpc.x_soft   = 0.12;             % [m]
mpc.r_du     = 5;                % peso nos incrementos de a_cmd (suaviza; evita chattering)
mpc.r_u      = 0.01;             % peso na magnitude de a_cmd
mpc.u_min    = -5;               % a_cmd mínima [m/s^2]
mpc.u_max    = 5;                % a_cmd máxima [m/s^2]
% --- algoritmo ---
mpc.eps      = 0.05;             % perturbação para a linearização [m/s^2]
mpc.alfa_n   = 0.7;              % filtro da inovação
mpc.iters    = 1;                % iterações do laço PNMPC (2 = ~9 ms no ESP32-S3)
mpc.varreduras = 30;             % varreduras do QP (Gauss-Seidel projetado)
mpc.v_max    = 0;                % (firmware) saturaÃ§Ã£o de velocidade no modelo: 0 = desligada
mpc.q_vs     = 0;                % peso da penalidade suave de velocidade (bancada: 1e4)
mpc.v_soft   = 0;                % |v| a partir do qual ela age [m/s] (bancada: 0,8*v_max do motor)
mpc.paralelo = 0;                % 0: y_n(k) = KAN(medição k-1, u(k-1)) | 1: modelo PARALELO (como na figura)
% --- custo terminal do EQUILÍBRIO: LQR (Riccati) da KAN linearizada em pé ---
mpc.r_lqr    = 0.51;             % R usado só na Riccati do custo terminal
if nargin < 1
    aqui = fileparts(mfilename('fullpath'));
    tmp = load(fullfile(aqui, 'kan_volta.mat'), 'kan'); kan = tmp.kan;
end
mpc.Pt = pnmpc_lqr_terminal(kan, mpc);
end
