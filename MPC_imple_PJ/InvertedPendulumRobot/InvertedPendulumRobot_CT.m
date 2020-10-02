function [dxdt, y, Ac, Bc, Cc, Dc] = pendulumCT(x, u, param)
% Continuous-time nonlinear dynamic model of two wheel inverted pendulum
%
% 4 states (x): 
%   Wheel angle (theta)
%   Pitch angle (psi)
%   Wheel angular velocity (theta_dot)
%   Angular velocity (psi_dot)
% 
% 2 inputs: (u)
%   Torque for left wheel (Fl)
%   Torque for right wheel (Fr)
%
% 2 outputs: (y)
%   theta and psi
%
% dxdt is the derivative of the states.
% [A B C D] are state space matrices linearized at the current operating point.
%
% Copyright 2020 The MathWorks, Inc.

%#codegen

% parameters
g = cast(param.g, 'like', x);               % 重力加速度 [m/sec2]
m = cast(param.m, 'like', x);               % 車輪合計質量 [kg]
R = cast(param.R, 'like', x);               % 車輪半径 [m]
Jw = cast(param.Jw, 'like', x);             % 車輪慣性モーメント [kgm2]
M = cast(param.M, 'like', x);               % 車体質量 [kg]
L = cast(param.L, 'like', x);               % 車輪中心から車体重心までの距離 [m]
Jpsi = cast(param.Jpsi, 'like', x);         % 車体慣性モーメント（ピッチ） [kgm2]
fm = cast(param.fm, 'like', x);             % 車体とDCモーター間の摩擦係数
fw = cast(param.fw, 'like', x);             % 車輪と路面間の摩擦係数
K = cast(param.K, 'like', x);               % 電流から駆動トルクへの変換係数[Nm/A]

% Obtain x, u and y
% x
psi = x(2);
theta_dot = x(3);
psi_dot = x(4);

% u
il = u(1);
ir = u(2);

% y
y = [x(1);x(2)];

% Compute dxdt
dxdt = calc_nlstate(Jpsi,Jw,K,L,M,R,fm,fw,g,il,ir,m,psi,psi_dot,theta_dot);

% Obtain A/B/C/D from Jacobian
% LTI
Ac = calc_Ac(Jpsi,Jw,K,L,M,R,fm,fw,g,il,ir,m,psi,psi_dot,theta_dot);
Bc = calc_Bc(Jpsi,Jw,K,L,M,R,m,psi);
Cc = cast(calc_Cc, 'like', x);
Dc = zeros(2,2, 'like', x);
end
