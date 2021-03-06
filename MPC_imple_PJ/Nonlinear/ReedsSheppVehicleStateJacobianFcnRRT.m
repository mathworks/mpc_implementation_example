function [A, B] = ReedsSheppVehicleStateJacobianFcnRRT(x, u, wb, ts)

% Variables
v = u(1);
delta = u(2);
q0 = x(3);
q3 = x(4);

% Linearize the state equations at the current condition
A = calc_Ac(ts,delta,q0,q3,v,wb);
B = calc_Bc(ts,delta,q0,q3,v,wb);

end