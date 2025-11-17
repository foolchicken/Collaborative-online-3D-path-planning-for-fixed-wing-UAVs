function [path, length, dubinsParam] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons)
% 计算三维dubins路径，水平方向和纵向分开计算，纵向使用OBVP问题求解
% 输入：起始姿态、目标姿态、飞行速度、飞行器动力学约束
% 注意航向角北偏东为正
% 输出：轨迹（如果不可行path为空）二维dubins参数

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));

% dubinsParam = dubins_core([startPose(1 : 2) pi / 2 - startPose(4)], [goalPose(1 : 2) pi / 2 - goalPose(4)], minR);
dubinsParam = dubins_core([startPose(1 : 2) startPose(4)], [goalPose(1 : 2) goalPose(4)], minR);
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

nseg = max(5, path2dimLength / minR * 4); % 路径分割的线段数量
path2dim = dubins_path_sample_many(dubinsParam, path2dimLength / nseg);
if size(path2dim, 1) == nseg
    path2dim = [path2dim; goalPose(1 : 2) goalPose(4)]; % 添加终点
end
path2dim(end, 1 : 2) = goalPose(1 : 2); % 消除小数计算误差

dist2dim = vecnorm(diff(path2dim(:, 1 : 2)), 2, 2); % 相邻点之间的二维路径距离
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist]; % 二维路径累加和

vz0 = speed * tan(startPose(5));
vzf = speed * tan(goalPose(5));
deltaV = vzf - vz0;

%tf = norm(goalPose(1 : 3) - startPose(1 : 3)) / speed * (1 + abs(goalPose(5) - startPose(5)) / 2); % 乘以放大系数保证安全
tf = hypot(path2dimLength, goalPose(3) - startPose(3))/speed;
t = tf * cumdist / path2dimLength;


x = [tf ^ 3 / 12 -tf ^ 2 / 4; tf ^ 2 / 4 -tf / 2] \ [deltaZ - vz0 * tf; deltaV];
alpha = x(1);
beta = x(2);
z = alpha * t .^ 3 / 12 - beta * t .^ 2 / 4 + vz0 * t + startPose(3);

% %t = linspace(0, tf, size(path2dim, 1));
% a = alpha * t - beta;
% max(abs(a))

path = [path2dim(:, 1 : 2) double(z)]; % 实际存在误差

dist3dim = vecnorm(diff(path), 2, 2); % 相邻点之间的三维路径距离
length = sum(dist3dim);




end








