function [E,F,G,constraintSlope,constraintIntercept] = obstacleComputeCustomConstraint(x,Measurement,detection,laneWidth,lanes)
%% Compute custom constraints for the obstacle.

%#codegen
egoX = x(1);
egoY = x(2);
dx = Measurement(1);
dy = Measurement(2);

% Compute constraints only if an obstacle is detected. Otherwsie, set
% constraint to lower road boundary (the inactive constraint).
if detection
    % If the ego car is behind of the obstacle 
    if (dx > 0)
        % If the ego car is already in the left adjacent lane, keep current
        % distance.
        if (dy < -3)    % -3 is margin width of the obstacle (3[m]=1.5*2[m])
            constraintSlope = 0;
            constraintIntercept = - dy;
        else
            % The ego car must be above the line formed from the ego car to
            % the obstacle for left passing.
            slope = (abs(dy) + 3) / dx;  % Constant is a bias of y location of the obstacle for obstacle avoidance
            constraintSlope = tan(atan2(slope,1));
            constraintIntercept = egoY - constraintSlope*egoX;
        end
    % If the ego car has passed the obstacle, use the inactive constraint
    % to go back to the center lane.
    else 
        constraintSlope = 0;
        constraintIntercept = -laneWidth*lanes/2;
    end
else
    constraintSlope = 0;
    constraintIntercept = -laneWidth*lanes/2;
end

%% Define constraint matrices.
E = [0 0;0 0;0 0];
F = [0 1 0 0;0 -1 0 0;constraintSlope -1 0 0]; 
G = [laneWidth*lanes/2;laneWidth*lanes/2;-1*constraintIntercept];
