close all
clear
clc
addpath('D:\Matlabproject\硕士毕业论文\2dim\Dubins-MP-RRT')
addpath('D:\Matlabproject\硕士毕业论文\3dim')
addpath('D:\Matlabproject\硕士毕业论文\3dim\MP-RRT')

startPose = [0 0 0 deg2rad(90) deg2rad(0)]; % 起始位置和姿态：xyz和方向角、俯仰角
goalPose = [1000 1000 200 deg2rad(90) deg2rad(0)];
speed = 30;

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);

%[pathseg, length] = DubinsPath([0 0 pi/2], [500 500 pi/2], 250);

n = 1000;
t = zeros(n, 1);
for i = 1 : n
    tic
    [path, dubinsParam] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons);
    t(i) = toc;
end
disp(['dubins法计算轨迹平均用时' num2str(sum(t) / n * 1000) '毫秒'])

figure
plot3(path(:, 1), path(:, 2), path(:, 3))
grid on
axis equal

load('D:\Matlabproject\硕士毕业论文\mpdata\dubins\mpdatav80.mat')
gridres = 500;
yawdisNum = 24;

for i = 1 : n
    tic
    motionPrimitive = GetMotionPrimitiveWithHigh(mpdataFTA, startPose, goalPose, dynamicCons, false);
    t(i) = toc;
end
disp(['运动基元法计算轨迹平均用时' num2str(sum(t) / n * 1000) '毫秒'])

path = motionPrimitive.path;
figure
plot3(path(:, 1), path(:, 2), path(:, 3))
grid on
axis equal

dynamicCons.vmin = speed;
dynamicCons.vmax = speed;
dynamicCons.yawmin = -2 * pi; % 偏航角
dynamicCons.yawmax = 2 * pi;
dynamicCons.Nxmin = -1; % 切向过载
dynamicCons.Nxmax = 1;
dynamicCons.Nzmin = -1 / cos(dynamicCons.rollmin); % 法向过载
dynamicCons.Nzmax = 1 / cos(dynamicCons.rollmax);
dynamicCons.drollmin = -2;
dynamicCons.drollmax = 2;
regionCons = [-200 5500; -200 5500; -200 5500]; % 飞行区域限制，依次为xyz的限制

% [state, control, cost] = SolveTPBVP3dim(startPose, goalPose, speed, regionCons, dynamicCons);
%
% figure
% plot3(state(:,1), state(:,2), state(:,3))
% grid on
% axis equal


% gridresxy = 500; % 水平方向网格分辨率
% gridresz = 100; % 高度方向网格分辨率
% yawdisNum = 8; % 方向角等分数量
% pitchdisNum = 6;
% databasePointNum = 11 ^ 2;

% tic
% mpDataBase3dim = GetDubinsMotionPrimitiveDataBase3dim(speed, gridresxy, gridresz, yawdisNum, pitchdisNum, databasePointNum, dynamicCons, 'database1');
% toc

% ShowDataBaseLayer(mpDataBase3dim(:,:,1))
% mpDataBase3dim(:,:,3)



