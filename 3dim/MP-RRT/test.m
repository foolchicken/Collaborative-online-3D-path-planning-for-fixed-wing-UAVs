addpath('D:\Matlabproject\dubins编队集结')
addpath('D:\Matlabproject\航迹规划算法\Dubins-RRT')
addpath('D:\Matlabproject\硕士毕业论文\3dim\Quick-Dubins-MP-RRT')
close all
clear
clc
%% TVBVP三维测试
startPose = [0 0 0 deg2rad(0) deg2rad(-10)]; % 起始位置和姿态：xyz和方向角、俯仰角
goalPose = [0 4000 0 deg2rad(0) deg2rad(10)];
speed = 60;

regionCons = [-200 5500; -200 5500; -1000 5500]; % 飞行区域限制，依次为xyz的限制

dynamicCons.vmin = speed;
dynamicCons.vmax = speed;
dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.yawmin = -2 * pi; % 偏航角
dynamicCons.yawmax = 2 * pi;
dynamicCons.Nxmin = -1; % 切向过载
dynamicCons.Nxmax = 1;
dynamicCons.Nzmin = 0.2; % 法向过载
dynamicCons.Nzmax = 2;
dynamicCons.drollmin = -2;
dynamicCons.drollmax = 2;

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));

tic
[state, control, time, cost] = SolveTPBVP3dim(startPose, goalPose, speed, regionCons, dynamicCons, true);
toc
disp(['飞行时间' num2str(time(end)) 's'])

x = state(:, 1);
y = state(:, 2);
z = state(:, 3);
trajectory = [x y z];
disp(['路径总长度' num2str(sum(vecnorm(diff(trajectory),2,2))) 'm'])

yaw = state(:, 4);
pitch = state(:, 5);
roll = state(:, 6);
v = state(:, 7);
nx = control(:, 1);
nz = control(:, 2);
drolldt = control(:, 3);

startPose(4) = pi/2 - startPose(4);
goalPose(4) = pi/2 - goalPose(4);

path = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons, true);
figure
plot3(x, y, z)
hold on
plot3(path(:, 1), path(:, 2), path(:, 3))
grid on
hold off
%axis equal
% reducedpath = DouglasPeucker([x y z], 5);
% figure
% plot3(reducedpath(:,1), reducedpath(:,2) ,reducedpath(:,3))
% grid on

%% 三维数据库测试

% gridresxy = 500; % 水平方向网格分辨率
% gridresz = 100; % 高度方向网格分辨率
% yawdisNum = 8; % 方向角等分数量
% pitchdisNum = 3;
% databasePointNum = 2 ^ 2;
%
% time = zeros(yawdisNum, 1);
% for i = 0 : yawdisNum - 1
%     goalPose(4) = i * 2 * pi / yawdisNum;
%     tic
%     [state, control, cost] = SolveTPBVP3dim(startPose, goalPose, speed, regionCons, dynamicCons, true);
%     time(i + 1) = toc;
%     x = state(:, 1);
%     y = state(:, 2);
%     z = state(:, 3);
%     trajectory = [x y z];
%
%     yaw = state(:, 4);
%     pitch = state(:, 5);
%     roll = state(:, 6);
%     v = state(:, 7);
%     nx = control(:, 1);
%     nz = control(:, 2);
%     drolldt = control(:, 3);
%
%     figure
%     plot3(x, y, z)
%     grid on
%     axis equal
%
% end

% tic
% mpDataBase = GetMotionPrimitiveDataBase3dim(speed, gridresxy, gridresz, yawdisNum, pitchdisNum, databasePointNum, regionCons, dynamicCons, 'database1');
% toc
% tic
% mpDataBase = GetMotionPrimitiveDataBase3dimFreeTA(speed, gridresxy, gridresz, yawdisNum, pitchdisNum, databasePointNum, regionCons, dynamicCons, 'databaseFTA1');
% toc

%mpDataBase(:,:,1)

% figure
% for k = 1 : size(mpDataBase, 3)
%     for i = 1 : size(mpDataBase, 1)
%         for j = 1 : size(mpDataBase, 2)
%             if isempty(mpDataBase{i, j, k}), continue; end
%             for n = 1 : size(mpDataBase{i, j, k}, 1)
%                 if isinf(mpDataBase{i, j, k}(n).cost), continue; end
%                 plot3(mpDataBase{i, j, k}(n).path(:, 1), mpDataBase{i, j, k}(n).path(:, 2), mpDataBase{i, j, k}(n).path(:, 3), 'LineWidth', 1);
%                 hold on
%             end
%         end
%     end
% end
% hold off
% axis equal
% grid on

% motionPrimitive = GetMotionPrimitive(mpDataBase, startPose, goalPose, v0, vf, gridres, thetadisNum, vdisNum, false);
% trajectory = motionPrimitive.trajectory;
% startPose = [0 0 0];
% goalPose = [2 -2 0];
% t = zeros(100, 1);
% for i = 1 : 1000
%     tic
%     motionPrimitive = GetMotionPrimitive(mpDataBase, startPose, goalPose, v0, vf, gridres, thetadisNum, vdisNum);
%     t(i) = toc;
% end
% disp(['运动基元法计算轨迹平均用时' num2str(sum(t) / 1000 * 1000) '毫秒'])
%
% for i = 1 : 1000
%     tic
%     dubConnObj = dubinsConnection();
%     dubConnObj.MinTurningRadius = 0.5;
%     [path, length] = connect(dubConnObj, startPose, goalPose);
%     t(i) = toc;
% end
% disp(['dubins法计算轨迹平均用时' num2str(sum(t) / 1000 * 1000) '毫秒'])
%
%
% figure
% plot(motionPrimitive.trajectory(:, 1), motionPrimitive.trajectory(:, 2))
% axis equal
% grid on


% [pathseg, length] = DubinsPath(startPose, goalPose, 5);
% show(pathseg.obj)
