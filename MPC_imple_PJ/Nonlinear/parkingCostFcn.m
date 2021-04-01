function J = parkingCostFcn(stage,x,u,dmv,p)
% Cost function

% parameters
ref =  p(1:numel(x));
Wx =   diag(p(numel(x)+1:2*numel(x)));
Wmv =  diag(p(2*numel(x)+1:2*numel(x)+numel(u)));
Wdmv = diag(p(2*numel(x)+numel(u)+1:2*numel(x)+2*numel(u)));

if stage == 1
    J = dmv'*Wdmv*dmv;
else
    J = (x-ref)'*Wx*(x-ref) + dmv'*(stage * Wdmv)*dmv;
end

end

% Copyright 2021 The MathWorks, Inc.