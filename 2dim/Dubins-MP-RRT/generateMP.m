% 生成基元数据库测试
close all
clear
clc

speed = 10;

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);

gridres = 5; % 网格分辨率
thetadisNum = 24; % 方向角等分数量
databasePointNum = 21 ^ 2; % 数据库离散点数量

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));

tic
mpDataBase = GetDubinsMotionPrimitiveDataBase2dim(speed, gridres, thetadisNum, databasePointNum, dynamicCons, 'mpdatav10');
toc

tic
GetDubinsMotionPrimitiveDataBase2dimFreeTA(speed, gridres, thetadisNum, databasePointNum, dynamicCons, 'mpdataFTAv10');
toc

%ShowDataBase(mpDataBase);

n = 1000;
t = zeros(n, 1);
for i = 1 : n
    tic
    motionPrimitive = GetMotionPrimitive(mpDataBase, [0 0 0], [1500 1500 deg2rad(45)], gridres, thetadisNum);
    t(i) = toc;
    path = motionPrimitive.path;
end
disp(['运动基元法计算轨迹平均用时' num2str(sum(t) / n * 1000) '毫秒'])

for i = 1 : n
    tic
    dubins_core([0 0 0], [1500 1500 deg2rad(45)], minR);
    t(i) = toc;
end
disp(['dubins法计算轨迹平均用时' num2str(sum(t) / n * 1000) '毫秒'])

% figure
% plot(path(:, 1), path(:, 2))
% axis equal
% grid on

