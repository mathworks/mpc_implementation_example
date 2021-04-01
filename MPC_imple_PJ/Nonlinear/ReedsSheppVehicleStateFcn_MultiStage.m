function dxdt = ReedsSheppVehicleStateFcn_MultiStage(x, u, p)

% Variables
v     = u(1);
delta = u(2);
q0    = x(3);
q3    = x(4);
wb    = p(1);
ts    = p(2);

dxdt = calc_nonlinear_f(ts,delta,q0,q3,v,wb);

end