function motionPrimitive = GetMotionPrimitiveWithHigh(dataBase, startPose, goalPose, dynamicCons, ifconsAnglef)
% 从数据库中提取运动基元并计算高度，输入二维数据库、起点、终点、速度、动力学约束、网格分辨率、航向角粒度是否约束终端角
% 一个姿势的组成：xyz yaw pitch，均为弧度制

if nargin == 4
    ifconsAnglef = true;
end

if ifconsAnglef
    mp2dim = GetMotionPrimitive(dataBase, [startPose(1 : 2) startPose(4)], [goalPose(1 : 2) goalPose(4)], ifconsAnglef);
else
    mp2dim = GetMotionPrimitive(dataBase, [startPose(1 : 2) startPose(4)], [goalPose(1 : 2) 0], ifconsAnglef); % 不约束终端角，查询的时候方位角设为0
end
if isempty(mp2dim) || isinf(mp2dim.cost)
    motionPrimitive.cost = inf;
    return
end

speed = dataBase.speed;
path2dim = mp2dim.path;
path2dimLength = mp2dim.cost;

deltaZ = goalPose(3) - startPose(3); % 高度变化量
if goalPose(3) > startPose(3) && path2dimLength * tan(dynamicCons.pitchmax) < deltaZ
    motionPrimitive.cost = inf;
    return
elseif  goalPose(3) < startPose(3) && path2dimLength * tan(dynamicCons.pitchmin) > deltaZ
    motionPrimitive.cost = inf;
    return
end

motionPrimitive.yaw0 = mp2dim.theta0;
motionPrimitive.yawf = mp2dim.thetaf;
motionPrimitive.pitch0 = startPose(5);

dist2dim = vecnorm(diff(path2dim(:, 1 : 2)), 2, 2); % 相邻点之间的二维路径距离
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist]; % 二维路径累加和

vz0 = speed * tan(startPose(5));

if ifconsAnglef
    pitchf = goalPose(5);
else
    pitchf = atan2(deltaZ, path2dimLength); % 不给定终端俯仰角暂时设为起点指向终点的角
end

tf = norm(goalPose(1 : 3) - startPose(1 : 3)) / speed * (1 + abs(pitchf - startPose(5)) / 2); % 乘以放大系数保证安全
t = tf * cumdist / path2dimLength;

if ifconsAnglef
    vzf = speed * sin(goalPose(5));
    deltaV = vzf - vz0;
    x = [tf ^ 3 / 6, -tf ^ 2 / 2; tf ^ 2 / 2, -tf] \ [deltaZ - vz0 * tf; deltaV];
    alpha = x(1);
    beta = x(2);
    motionPrimitive.pitchf = goalPose(5);
else
    alpha = 3 * (vz0 * tf - deltaZ) / tf ^ 3;
    beta = alpha * tf;
    vzf = alpha * tf ^ 2 / 2 - beta * tf + vz0;
    motionPrimitive.pitchf = asin(vzf / speed);
end

z = alpha * t .^ 3 / 6 - beta * t .^ 2 / 2 + vz0 * t + 0;
path = [path2dim(:, 1 : 2) z]; % 实际存在误差

dist3dim = vecnorm(diff(path), 2, 2); % 相邻点之间的三维路径距离
length = sum(dist3dim);

motionPrimitive.cost = length;
motionPrimitive.path = path;

end








