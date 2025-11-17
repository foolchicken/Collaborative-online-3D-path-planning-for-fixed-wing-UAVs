% 生成基元数据库测试
close all
clear
clc

addpath('D:\Matlabproject\硕士毕业论文\2dim\Quick-Dubins-MP-RRT')
speed = 140;

dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);

gridres = 1000; % 网格分辨率
thetadisNum = 24; % 方向角等分数量
databasePointNum = 21 ^ 2; % 数据库离散点数量

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));

tic
mpdata = GetDubinsMotionPrimitiveDataBase2dim(speed, gridres, thetadisNum, databasePointNum, dynamicCons,...
    'D:\Matlabproject\硕士毕业论文\mpdata\dubins\mpdatav140');
toc

tic
GetDubinsMotionPrimitiveDataBase2dimFreeTA(speed, gridres, thetadisNum, databasePointNum, dynamicCons,...
    'D:\Matlabproject\硕士毕业论文\mpdata\dubins\mpdataFTAv140');
toc


