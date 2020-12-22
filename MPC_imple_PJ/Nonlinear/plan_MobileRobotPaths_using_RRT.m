function pthObj = plan_MobileRobotPaths_using_RRT(Ts, Tf, start_pos, goal_pos)
%% Rapidly-exploring Random Tree (RRT) を用いたパスプランを行います
%% 占有グリッドを読み込む
load("office_area_gridmap.mat", "occGrid");
show(occGrid);

%% スタート、ゴール位置を表示する
hold on;
plot(start_pos(1), start_pos(2), 'ro');
plot(goal_pos(1), goal_pos(2), 'mo');

%% スタート、ゴール時の向くべき姿勢角（ヨー角）を表示する
r = 0.5;
plot([start_pos(1), start_pos(1) + r*cos(start_pos(3))], [start_pos(2), start_pos(2) + r*sin(start_pos(3))], 'r-' );
plot([goal_pos(1), goal_pos(1) + r*cos(goal_pos(3))], [goal_pos(2), goal_pos(2) + r*sin(goal_pos(3))], 'm-' );
hold off;

%% 状態空間を定義する
bounds = [occGrid.XWorldLimits; occGrid.YWorldLimits; [-pi pi]];

vehicle_ss = stateSpaceReedsShepp(bounds);
vehicle_ss.MinTurningRadius = 0.4;

%% パスプラン
stateValidator = validatorOccupancyMap(vehicle_ss); 
stateValidator.Map = occGrid;
stateValidator.ValidationDistance = 0.05;

planner = plannerRRT(vehicle_ss, stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 30000;

planner.GoalReachedFcn = @check_MobileRobotPaths_If_Goal;

rng(0,'twister');

[pthObj, solnInfo] = plan(planner, start_pos, goal_pos);

%% 結果を表示する
figure;
show(occGrid);
hold on;
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

interpolate(pthObj, Tf / Ts);
plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2);

plot(start_pos(1), start_pos(2), 'ro');
plot(goal_pos(1), goal_pos(2), 'mo');
hold off;

end