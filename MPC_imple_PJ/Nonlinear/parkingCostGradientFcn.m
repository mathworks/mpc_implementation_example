function [Gx, Gmv, Gdmv] = parkingCostGradientFcn(stage,x,u,dmv,p)
% Analytical gradient of the cost function

% parameters
ref =  p(1:numel(x));
Wx =   diag(p(numel(x)+1:2*numel(x)));
Wmv =  diag(p(2*numel(x)+1:2*numel(x)+numel(u)));
Wdmv = diag(p(2*numel(x)+numel(u)+1:2*numel(x)+2*numel(u)));

Gmv = zeros(2,1);
if stage == 1
    Gx = zeros(4,1);
    Gdmv = 2*Wdmv*dmv;
else
    Gx = 2*Wx*(x-ref);
    Gdmv = 2*(stage * Wdmv)*dmv;
end

end

% Copyright 2021 The MathWorks, Inc.