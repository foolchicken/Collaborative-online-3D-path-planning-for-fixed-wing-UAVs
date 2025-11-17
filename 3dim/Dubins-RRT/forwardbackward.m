% MATLAB Code for Forward-Backward Iteration Method
close all
clear
clc

% 系统参数
A = [0 1; 0 0];        % 状态矩阵
B = [0; 1];            % 控制矩阵
T = 1.0;               % 终端时间
N = 100;               % 时间步数
dt = T / N;            % 时间步长
umax = 1.0;            % 控制约束

% 初始条件
x0 = [0; 0];           % 初始状态
xT = [1; 0];           % 终端状态

% 初始化变量
x = zeros(2, N+1);     % 状态 x
lambda = zeros(2, N+1); % 伴随变量 lambda
u = zeros(1, N);       % 控制量 u

% 设定初值
x(:,1) = x0;           % 初始状态
lambda(:,end) = [0; 0]; % 伴随变量终端条件

% 迭代求解
max_iter = 100;        % 最大迭代次数
tolerance = 1e-4;      % 收敛容差

for iter = 1:max_iter
    % 前向积分状态方程
    for k = 1:N
        x(:,k+1) = x(:,k) + dt * (A * x(:,k) + B * u(k));
    end
    
    % 后向积分伴随方程
    for k = N:-1:1
        lambda(:,k) = lambda(:,k+1) - dt * (A' * lambda(:,k+1));
    end
    
    % 更新控制量
    u_old = u; % 保存上一轮控制量用于收敛性检查
    for k = 1:N
        u(k) = -B' * lambda(:,k);  % 最优控制律
        % 投影到控制约束范围内
        u(k) = max(min(u(k), umax), -umax);
    end
    
    % 检查收敛性
    if norm(u - u_old, Inf) < tolerance
        fprintf('Converged after %d iterations.\n', iter);
        break;
    end
end

% 显示结果
t = linspace(0, T, N+1); % 时间向量
figure;
subplot(3,1,1);
plot(t, x(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, x(2,:), 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('States'); legend('x_1', 'x_2');
title('States over time'); grid on;

subplot(3,1,2);
plot(t(1:end-1), u, 'k', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Control (u)');
title('Control over time'); grid on;

subplot(3,1,3);
plot(t, lambda(1,:), 'r--', 'LineWidth', 1.5); hold on;
plot(t, lambda(2,:), 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Adjoint Variables (\lambda)');
legend('\lambda_1', '\lambda_2');
title('Adjoint Variables over time'); grid on;

% 显示最终状态
disp('Final state:');
disp(x(:,end));
disp('Target state:');
disp(xT);
