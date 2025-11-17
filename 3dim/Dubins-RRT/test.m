close all
clear
clc

startPose = [0 0 0 deg2rad(0) deg2rad(10)]; % 起始位置和姿态：xyz和方向角、俯仰角
goalPose = [500 500 150 deg2rad(15) deg2rad(0)];
speed = 60;

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);

% [path, dubinsParam] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons);
% 
% 
% figure
% plot3(path(:,1), path(:,2), path(:,3))
% grid on
% axis equal

dynamicCons.vmin = speed*dynamicCons.pitchmin;
dynamicCons.vmax = speed*dynamicCons.pitchmax;
dynamicCons.amin = -20;
dynamicCons.amax = 20;

length = 1500;
hf = length*tan(dynamicCons.pitchmax);
hf = 800;
tf = hypot(length,hf)/speed;
tf = 35;
hmin = -10;
hmax = hf+100;
h0 = 0;
v0 = 0;
vf = 0;

tic
[h, v, a, t] = SolveHTPBVP(h0, hf, v0, vf, hmin, hmax, dynamicCons, tf);
toc

plot(t, h);
max(a)



