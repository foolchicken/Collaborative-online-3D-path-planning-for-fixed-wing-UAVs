close all
clear
clc
addpath('D:\Matlabproject\硕士毕业论文\3dim\Dubins-MP-RRT')
startPose = [0 0 100 deg2rad(90) deg2rad(0)]; % 起始位置和姿态：xyz和方向角、俯仰角
goalPose = [1000 1000 800 deg2rad(90) deg2rad(0)];
speed = 30;

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);


[path, dubinsParam] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons);

dim3pathlen = sum(vecnorm(diff(path), 2, 2));
dim2pathlen = sum(vecnorm(diff(path(:, 1 : 2)), 2, 2));


% startPose = [0 0 0 deg2rad(90) deg2rad(0)]; % 起始位置和姿态：xyz和方向角、俯仰角
% goalPose = [1000 400 600 deg2rad(90) deg2rad(0)];
% [path1, dubinsParam1] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons);
% 
% dim3pathlen1 = sum(vecnorm(diff(path1), 2, 2));
% dim2pathlen1 = sum(vecnorm(diff(path1(:, 1 : 2)), 2, 2));

figure
plot3(path(:, 1), path(:, 2), path(:, 3))
% hold on
% plot3(path1(:, 1), path1(:, 2), path1(:, 3))
hold off
grid on
axis equal

% dim2pathlen - dim2pathlen1
% hypot(dim2pathlen, 600) - hypot(dim2pathlen1, 600)
% dim3pathlen - dim3pathlen1


%path = AdjDubinsPathLength([0 0], deg2rad(90), [1000 1000], deg2rad(90), 200, 100, 'R', true);

tic
path = AdjDubinsPathLength3dimh(startPose, goalPose, speed, dynamicCons, 1200, 'L');
toc
figure
plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 1.5)
axis equal
grid on
axis equal


dim3pathlennew = sum(vecnorm(diff(path), 2, 2));
dim3pathlennew - dim3pathlen - 1200


% [path, newlength] = AdjDubinsPathLength([0 0], pi/2, [900 500], 0.2618, 343, 17, 'L', true);
% 

