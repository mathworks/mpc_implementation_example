function [Ad,Bd,Cd,Dd,U,Y,X,DX] = calc_Discrete_SS_for_Adaptive( ...
    I, K_f, K_r, l_f, l_r, m, ts, x, u, div_min)

delta   = u(1);
theta   = x(3);
r       = x(4);
beta    = x(5);
V       = x(6);
a       = u(2);

Ac = cast(calc_Ac_azd(I,K_f,K_r,V,beta,delta,l_f,l_r,m,r,theta,div_min), 'like', x);
Bc = cast(calc_Bc_azd(I,K_f,V,l_f,m,div_min), 'like', x);
Cc = cast(calc_Cc, 'like', x);

nx = size(Ac, 1);
nu = size(Bc, 2);
ny = size(Cc, 1);

M = expm([[Ac Bc]*ts; zeros(nu,nx+nu)]);
Ad = M(1:nx,1:nx);
Bd = M(1:nx,nx+1:nx+nu);

Cd = Cc;
Dd = cast(zeros(ny, nu), 'like', x);

% ノミナル状態
X = x;
U = u;
Y = Cc * x;
DX = Ad * x + Bd * u - x;

end

