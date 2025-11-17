function [path, length, dubinsParam] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons)
% 计算三维dubins路径，水平方向和纵向分开计算，纵向使用OBVP问题求解
% 输入：起始姿态、目标姿态、飞行速度、飞行器动力学约束
% 注意航向角北偏东为正
% 输出：轨迹（如果不可行path为空）二维dubins参数

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));

dubinsParam = dubins_core([startPose(1 : 2) pi / 2 - startPose(4)], [goalPose(1 : 2) pi / 2 - goalPose(4)], minR);
path2dimLength = dubins_length(dubinsParam); % 二维dubins路径长度

deltaZ = goalPose(3) - startPose(3); % 高度变化量
if goalPose(3) > startPose(3) && path2dimLength * tan(dynamicCons.pitchmax) < deltaZ
    path = [];
    length = inf;
    return
elseif  goalPose(3) < startPose(3) && path2dimLength * tan(dynamicCons.pitchmin) > deltaZ
    path = [];
    length = inf;
    return
end
if path2dimLength == 0
    path = startPose;
    length = 0;
    return
end
seglen = minR / 10;
path2dim = dubins_path_sample_many(dubinsParam, path2dimLength / seglen);
if ~isequal(path2dim(1 : 2), goalPose(1 : 2))
    path2dim = [path2dim; goalPose(1 : 2) goalPose(4)]; % 添加终点
end

vz0 = speed * tan(startPose(5));
vzf = speed * tan(goalPose(5));
deltaV = vzf - vz0;

tf = norm(goalPose(1 : 3) - startPose(1 : 3)) / speed * 1.1; % 乘以放大系数保证安全
t = linspace(0, tf, size(path2dim, 1));

x = [tf ^ 3 / 12 -tf ^ 2 / 4; tf ^ 2 / 4 -tf / 2] \ [deltaZ- vz0*tf; deltaV];
alpha = x(1);
beta = x(2);
z = alpha * t .^ 3 / 12 - beta * t .^ 2 / 4 + vz0 * t + startPose(3);

path = [path2dim(:, 1 : 2) z']; % 实际存在误差

%length = hypot(path2dimLength, deltaZ); % 路径长度使用平方和近似

length = path2dimLength;
end








