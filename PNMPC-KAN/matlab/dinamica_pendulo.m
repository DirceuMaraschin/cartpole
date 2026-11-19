function ds = dinamica_pendulo(s, u, F_carro, F_ponta, p)
%DINAMICA_PENDULO  Modelo não linear do pêndulo invertido sobre carrinho.
%   ds = dinamica_pendulo(s, u, F_carro, F_ponta, p)
%   s       = [x; dx; th; dth]
%   u       força do motor no carrinho [N]
%   F_carro perturbação: força horizontal extra no carrinho [N]
%   F_ponta perturbação: força horizontal na ponta da haste [N]
%
%   Equações de Lagrange (th = 0 em pé):
%   (M+m) x'' + m l cos(th) th'' = u + F_carro + F_ponta - b x' + m l th'^2 sin(th)
%   m l cos(th) x'' + (I+m l^2) th'' = m g l sin(th) - c th' + F_ponta Lh cos(th)

dx  = s(2);
th  = s(3);
dth = s(4);
ct = cos(th); st = sin(th);

A11 = p.M + p.m;       A12 = p.m*p.l*ct;
A21 = A12;             A22 = p.I + p.m*p.l^2;
r1 = u + F_carro + F_ponta - p.b*dx + p.m*p.l*dth^2*st;
r2 = p.m*p.g*p.l*st - p.c*dth + F_ponta*p.Lh*ct;

det = A11*A22 - A12*A21;
ddx  = ( A22*r1 - A12*r2)/det;
ddth = (-A21*r1 + A11*r2)/det;

ds = [dx; ddx; dth; ddth];
end
