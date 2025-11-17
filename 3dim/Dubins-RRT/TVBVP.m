% Z轴方向上的两点边界最优控制问题
% 根据唯一tf可以确定一个解
close all
clear
clc


z0 = 0;
zf = 0;
v0 = -10;
vf = 10;

deltaZ = zf - z0;
deltaV = vf - v0;

tf = 20;

x = [tf ^ 3 / 6 -tf ^ 2 / 2; tf ^ 2 / 2 -tf] \ [deltaZ - v0*tf;deltaV];
alpha = x(1);
beta = x(2);
% alpha = 3 * (v0 * tf - deltaZ) / tf ^ 3;
% beta = alpha * tf;


tstep = 0.001;
t = 0 : tstep : tf;
% fmincon
% quadprog
% bvp4c

% alpha = -6*deltaZ/tf^3;
% beta = -6*deltaZ/tf^2;

z = alpha * t .^ 3 / 6 - beta * t .^ 2 / 2 + v0 * t + z0;
v = alpha * t .^ 2 / 2 - beta * t + v0;
a = alpha * t - beta;
% max(a)
% amin = -1;
% amax = 1;
% a(a<amin) = amin;
% a(a>amax) = amax;

v(1) = v0;
z(1) = z0;
for i=2:numel(t)
    v(i) = v(i-1) + a(i-1)*tstep;
    z(i) = z(i-1) + v(i-1)*tstep;
end

figure
plot(t, z, 'LineWidth', 2)
title('位置时间图像')

figure
plot(t, v, 'LineWidth', 2)
title('速度时间图像')

figure
plot(t, a, 'LineWidth', 2)
title('加速度时间图像')


%%
% 清除工作空间
clear;

% 定义系统矩阵 A 和 B
A = [0 1; 0 0]; % 状态矩阵
B = [0; 1]; % 控制矩阵

% 定义权重矩阵 Q 和 R
Q = [1 0; 0 1]; % 状态权重
R = 0.1; % 控制权重

% 定义时间范围
t_f = 50; % 终端时间
N = 50; % 离散时间步数
t = linspace(0, t_f, N); % 时间向量
deltat = t(2) - t(1);

% 初始和终端状态
x0 = [0; 10]; % 起点状态
xf = [0; 10]; % 终点状态

% 控制量约束
u_min = -5;
u_max = 5;

% 优化变量
x = zeros(2, N); % 状态矩阵
u = zeros(1, N - 1); % 控制矩阵

% 初始条件
x(:, 1) = x0;

% 优化选项
options = optimoptions('fmincon', 'Display', 'off');

% 目标函数
J = @(u) cost_function(u, x0, xf, A, B, Q, R, N, deltat);

% 约束函数
constraints = @(u) state_constraints(u, x, A, B, N, xf, deltat);
tic
% 求解优化问题
u_opt = fmincon(J, u, [], [], [], [], repmat(u_min, 1, N - 1), repmat(u_max, 1, N - 1), constraints, options);
toc
% 计算优化后的状态
for k = 1 : N - 1
    x(:, k + 1) = x(:, k) + (A * x(:, k) + B * u_opt(k)) * (t(2) - t(1)); % 欧拉法
end

% 绘图
figure;
subplot(2, 1, 1);
plot(t, x);
xlabel('时间 (s)');
ylabel('状态');
title('状态响应');

subplot(2, 1, 2);
stairs(t(1 : end - 1), u_opt);
xlabel('时间 (s)');
ylabel('控制输入');
title('控制输入');

% 直接打靶法
function J = cost_function(u, x0, xf, A, B, Q, R, N, deltat)
% 计算成本函数
x = zeros(2, N); % 状态矩阵
x(:, 1) = x0; % 设置初始状态
for k = 1 : N - 1
    x(:, k + 1) = x(:, k) + (A * x(:, k) + B * u(k)) * deltat; % 欧拉法
end
J = 0;
for k = 1 : N - 1
    J = J + u(k)' * R * u(k);
end
end

function [c, ceq] = state_constraints(u, x, A, B, N, xf, deltat)
% 状态约束：终点状态
x_temp = zeros(2, N); % 状态矩阵
x_temp(:, 1) = x(:, 1); % 初始状态
for k = 1 : N - 1
    x_temp(:, k + 1) = x_temp(:, k) + (A * x_temp(:, k) + B * u(k)) * deltat; % 欧拉法
end
c = []; % 不等式约束
ceq = x_temp(:, end) - xf; % 等式约束：终点状态
end


