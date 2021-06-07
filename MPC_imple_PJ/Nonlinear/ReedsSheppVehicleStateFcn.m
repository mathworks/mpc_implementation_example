function x_next = ReedsSheppVehicleStateFcn(x, u, wb, ts)

% 変数
v     = u(1);
delta = u(2);
px    = x(1);
py    = x(2);
q0    = x(3);
q3    = x(4);

x_next = calc_nonlinear_f(ts,delta,px,py,q0,q3,v,wb);

end