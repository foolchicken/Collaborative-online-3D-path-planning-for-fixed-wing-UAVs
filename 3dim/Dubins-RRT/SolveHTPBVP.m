function [h, v, a, t] = SolveHTPBVP(h0, hf, v0, vf, hmin, hmax, dynamicCons, tf)
% 调用GPOPSII求解高度方向的两点边值问题
% 输入起终点高度、起终点速度、高度范围、动力学约束、终点时间
% 输出一条轨迹和性能指标
% 状态量：位置、速度
% 控制量：加速度

t0 = 0;
tfmin = tf;
tfmax = tf;

%% 设置约束条件
bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tfmin;
bounds.phase.finaltime.upper = tfmax;
bounds.phase.initialstate.lower = [h0 v0]; % 起始时刻状态上下限
bounds.phase.initialstate.upper = [h0 v0];
bounds.phase.finalstate.lower = [hf vf];
bounds.phase.finalstate.upper = [hf vf];
bounds.phase.state.lower = [hmin, dynamicCons.vmin]; % 状态量约束
bounds.phase.state.upper = [hmax, dynamicCons.vmax];
bounds.phase.control.lower = [dynamicCons.amin]; % 控制量约束
bounds.phase.control.upper = [dynamicCons.amax];
bounds.phase.integral.lower = 0; % 积分约束的最大最小值
bounds.phase.integral.upper = 100;

%% 设置问题的初步猜测
guess.phase.time = [t0; (tfmin + tfmax) / 2]; % 长度为Mp的列向量，Mp为时间猜测中使用的值的数量
guess.phase.state = [h0 v0; hf vf]; % 列数等于变量问题维数
% guess.phase.control = [uMin; uMax]; % 列数等于控制量u的维数
guess.phase.control = [0, 0; 0, 0];
guess.phase.integral = 10; % 行向量，长度为相位积分的数量

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
setup.mesh.maxiteration = 5; % 最大迭代次数
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
%disp(['性能指标为',num2str(cost)]);
h = output.result.solution.phase.state(:, 1);
v = output.result.solution.phase.state(:, 2);

a = output.result.solution.phase.control(:, 1);
t = output.result.solution.phase.time;

end


function phaseout = DynimicsModel(input)
% 动力学模型

h = input.phase.state(:, 1);
v = input.phase.state(:, 2);

a = input.phase.control(:, 1);

phaseout.dynamics = [v, a];
phaseout.integrand = 0.5 * a .^ 2; % 被积函数
%phaseout.integrand = ones(size(a));
end

function output = CostFuncion(input)
% 性能指标函数
output.objective = input.phase.integral;

end
