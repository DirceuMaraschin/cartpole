function p = parametros_planta()
%PARAMETROS_PLANTA  Parâmetros físicos do pêndulo invertido sobre carrinho.
%   Edite aqui para casar com a sua bancada / com o modelo usado no
%   treinamento da rede neural.
%
%   Convenções (as mesmas do Arduino):
%     x    posição do carrinho [m]   (0 = meio do trilho, + para a direita)
%     dx   velocidade do carrinho [m/s]
%     th   ângulo do pêndulo [rad]   (0 = em pé, + inclinado para a direita)
%     dth  velocidade angular [rad/s]
%     u    força do motor no carrinho [N] (+ empurra para a direita)

p.M  = 0.5;      % massa do carrinho [kg]
p.m  = 0.2;      % massa do pêndulo [kg]
p.l  = 0.3;      % distância pivô -> centro de massa [m]
p.Lh = 2*p.l;    % comprimento total da haste [m] (onde atua o "empurrão")
p.I  = 0.006;    % momento de inércia em torno do CM [kg m^2]
p.b  = 0.1;      % atrito viscoso do carrinho [N s/m]
p.c  = 0.001;    % atrito viscoso no pivô [N m s/rad]
p.g  = 9.81;     % gravidade [m/s^2]

p.L_trilho = 0.60;  % COMPRIMENTO ÚTIL DO TRILHO [m] -> carrinho anda de -0.30 a +0.30
p.F_max    = 10;    % saturação do atuador [N]
end
