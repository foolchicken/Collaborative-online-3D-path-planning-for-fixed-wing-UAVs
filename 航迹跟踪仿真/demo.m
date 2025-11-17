% 实践证明L1制导律并不严格遵循航迹内切外切，具体飞行效果取决于L1的长度和判过点距离
% 判过点距离越小，飞行器过点越晚，外切的部分越长
% 判过点距离越大，飞行器过点越早，内切的部分越长，并且航迹点间距相较于转弯半径较大时内切效果越明显
% L1距离越大，震荡越小，上升时间越长
% 为了实现较好的内切效果，建议判过点距离大于等于L1距离
addpath('D:\Matlabproject\mydubins')
addpath('D:\Matlabproject\硕士毕业论文\3dim\Quick-Dubins-MP-RRT')
close all
clear
clc

speed = 140;
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = deg2rad(-30);
dynamicCons.pitchmax = deg2rad(30);
turingR = speed ^ 2 / 9.8 / tan(dynamicCons.rollmax) * 1.1;

% 定义初始和终止状态
startPose = [0 0 6000 deg2rad(90) 0];
goalPose = [10000 8800 5500 deg2rad(90) 0];
L1D = 1 / sqrt(2);
L1P = 60;
L1 = L1P * L1D * speed / pi;
TransitionRadius = 500;

waypoints = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons);
initialState = [startPose(1 : 3) speed, startPose(4 : 5)]; % 初始状态，位置、速度、航向角、俯仰角
state = UAVPathFollowSimulate(waypoints, initialState, L1, TransitionRadius, dynamicCons);
disp(['飞行时间 ' num2str(state(end,end)) ' s'])
disp(['终点误差 ' num2str(norm(state(end, 1 : 3) - waypoints(end, 1 : 3))) ' m'])

figure
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'b-*')
hold on
plot3(state(:, 1), state(:, 2), state(:, 3), 'LineWidth',1.5)
hold off
axis equal
grid on


figure
hold on
plot(state(:,end), rad2deg(state(:,5)), 'LineWidth', 1.5, 'DisplayName', '偏航角')
plot(state(:,end), -rad2deg(state(:,6)), 'LineWidth', 1.5, 'DisplayName', '俯仰角')
plot(state(:,end), rad2deg(state(:,7)), 'LineWidth', 1.5, 'DisplayName', '滚转角')
hold off
legend

state(end,3)
rad2deg(state(end,5))
rad2deg(state(end,6))
