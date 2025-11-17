function [state, control, time, cost] = SolveTPBVP3dim(startPose, goalPose, speed, regionCons, dynamicCons, ifconsTerminalAngle)
% 调用GPOPSII求解三维空间的两点边值问题，获取一个运动基元
% 输入起始姿势、目标姿势、起始速度、目标速度、区域限制、动力学约束
% 输出状态量、控制量、性能指标
% 控制量选择：Nx Nz drolldt
% 状态量选择：x y z yaw pitch roll v

if nargin == 5
    ifconsTerminalAngle = true;
end

v0 = speed;
vf = speed;
t0 = 0;
xmin = regionCons(1, 1);
xmax = regionCons(1, 2);
ymin = regionCons(2, 1);
ymax = regionCons(2, 2);
zmin = regionCons(3, 1);
zmax = regionCons(3, 2);
tfmin = norm(startPose(1 : 3) - goalPose(1 : 3)) / dynamicCons.vmax;
tfmax = tfmin * 5;

%% 设置约束条件
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfmin;
bounds.phase.finaltime.upper = tfmax;
bounds.phase.initialstate.lower = [startPose 0 v0]; % 起始时刻状态上下限
bounds.phase.initialstate.upper = [startPose 0 v0];
bounds.phase.finalstate.lower = [goalPose 0 vf];
bounds.phase.finalstate.upper = [goalPose 0 vf];
if ~ifconsTerminalAngle % 不给定终端角约束
    bounds.phase.finalstate.lower(4 : 6) = [dynamicCons.yawmin, dynamicCons.pitchmin, dynamicCons.rollmin];
    bounds.phase.finalstate.upper(4 : 6) = [dynamicCons.yawmax, dynamicCons.pitchmax, dynamicCons.rollmax];
end
bounds.phase.state.lower = [xmin, ymin, zmin, dynamicCons.yawmin, dynamicCons.pitchmin, dynamicCons.rollmin, dynamicCons.vmin]; % 状态量约束
bounds.phase.state.upper = [xmax, ymax, zmax, dynamicCons.yawmax, dynamicCons.pitchmax, dynamicCons.rollmax, dynamicCons.vmax];
bounds.phase.control.lower = [dynamicCons.Nxmin dynamicCons.Nzmin dynamicCons.drollmin]; % 控制量约束
bounds.phase.control.upper = [dynamicCons.Nxmax dynamicCons.Nzmax dynamicCons.drollmax];
bounds.phase.integral.lower = 0; % 积分约束的最大最小值
bounds.phase.integral.upper = 1e10;

%% 设置问题的初步猜测
guess.phase.time = [t0; (tfmin + tfmax) / 2]; % 长度为Mp的列向量，Mp为时间猜测中使用的值的数量
guess.phase.state = [startPose 0 v0; goalPose 0 vf]; % 列数等于变量问题维数
if ~ifconsTerminalAngle % 不给定终端角约束
    guess.phase.state(2, 4) = atan2(goalPose(1) - startPose(1), goalPose(2) - startPose(2));
    guess.phase.state(2, 5) = atan2(goalPose(3) - startPose(3), norm(goalPose(1 : 2) - startPose(1 : 2)));
end
guess.phase.control = [0, 1, 0; 0, 1, 0];
guess.phase.integral = 100; % 行向量，长度为相位积分的数量

%% 求解结构体设置
setup.name = 'OBVP-Problem';
setup.functions.continuous = @DynimicsModel;
setup.functions.endpoint = @CostFuncion;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'snopt';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
% setup.scales.method = 'automatic-bounds';
setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-3; % 网格精度
setup.mesh.maxiteration = 15; % 最大迭代次数
setup.mesh.colpointsmin = 3;
setup.mesh.colpointsmax = 10;
setup.mesh.phase.colpoints = 4 * ones(1, 10);
setup.mesh.phase.fraction = 0.1 * ones(1, 10);
setup.method = 'RPMintegration';
setup.display = false;

output = gpops2(setup);

if output.result.nlpinfo ~=0 && output.result.nlpinfo ~= 1
    disp('此情况无解')
    cost = inf;
else
    cost = output.result.solution.phase.integral;
end

state = output.result.solution.phase.state;
control = output.result.solution.phase.control;
time = output.result.solution.phase.time;

end


function phaseout = DynimicsModel(input)
% 动力学模型

x = input.phase.state(:, 1);
y = input.phase.state(:, 2);
z = input.phase.state(:, 3);
yaw = input.phase.state(:, 4);
pitch = input.phase.state(:, 5);
roll = input.phase.state(:, 6);
v = input.phase.state(:, 7);

dxdt = v .* cos(pitch) .* sin(yaw); % 注意是东北天坐标系
dydt = v .* cos(pitch) .* cos(yaw);
dzdt = v .* sin(pitch);

Nx = input.phase.control(:, 1);
Nz = input.phase.control(:, 2);
drolldt = input.phase.control(:, 3);

dvdt = 9.8 * (Nx - sin(pitch));
dpitchdt = 9.8 ./ v .* (Nz .* cos(roll) - cos(pitch));
dyawdt = 9.8 * Nz .* sin(roll) ./ (v .* cos(pitch));
%dyawdt = 9.8./v.*tan(roll);

phaseout.dynamics = [dxdt, dydt, dzdt, dyawdt, dpitchdt, drolldt, dvdt];
%phaseout.integrand = 1 + 0.5 * dyawdt .^ 2 + 0.5 * dpitchdt .^ 2;% + 0.5 * drolldt .^ 2; % 被积函数
%phaseout.integrand = 1 +  0.1 * dzdt .^ 2;
%phaseout.integrand = 1 +  0.5 * dpitchdt .^ 2;
phaseout.integrand = 1 + 0.5 * Nx .^ 2 + 0.5 * Nz .^ 2; % 总能量最小
%phaseout.integrand = 1 + 0.5*Nz .^ 2;
%phaseout.integrand = ones(size(Nz)); % 总时间最小

end

function output = CostFuncion(input)
% 性能指标函数
output.objective = input.phase.integral;

end
