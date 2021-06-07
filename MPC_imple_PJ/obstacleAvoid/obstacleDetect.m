function detection = obstacleDetect(Measurement, NumDetections, DetectionDistance)
% Detect when the vehicle sees an obstacle.
%#codegen
dx = Measurement(1);
dy = Measurement(2);
dist2Obstacle   = sqrt( dx^2 + dy^2 );
flagCloseEnough = (dist2Obstacle < DetectionDistance);
detection = ( flagCloseEnough && (NumDetections > 0) );