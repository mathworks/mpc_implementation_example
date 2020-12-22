function dxdt = ReedsSheppVehicleStateFcn(x, u, wb, ts)

v     = u(1);
delta = u(2);
q0    = x(3);
q3    = x(4);

dxdt = calc_nonlinear_f(ts,delta,q0,q3,v,wb);

end