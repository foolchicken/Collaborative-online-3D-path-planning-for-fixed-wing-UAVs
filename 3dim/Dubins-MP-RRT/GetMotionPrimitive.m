function motionPrimitive = GetMotionPrimitive(dataBase, startPose, goalPose, ifconsthetaf)
% 从数据库中提取运动基元

gridres = dataBase.gridres;
thetadisNum = dataBase.thetadisNum;
if nargin == 3
    ifconsthetaf = true;
end
if ~ifconsthetaf
    goalPose(3) = 0; % 算法保护，虽然不约束目标的终端角
end

% startPose(3) = wrapTo2Pimy(startPose(3)); % 角度规范化
% goalPose(3) = wrapTo2Pimy(goalPose(3));

thetares = 2 * pi / thetadisNum;
goalPose(1 : 2) = goalPose(1 : 2) - startPose(1 : 2); % 先平移到原点

if goalPose(1) == 0 && goalPose(2) == 0
    motionPrimitive = [];
    return
end

if goalPose(1) >= 0 && goalPose(2) >= 0 % 目标第一象限内
    theta0ind = startPose(3) / thetares + 1;
    thetafind = goalPose(3) / thetares + 1;
    xind = goalPose(1) / gridres + 1;
    yind = goalPose(2) / gridres + 1;
elseif goalPose(1) >= 0 && goalPose(2) < 0 % 第四象限
    theta0ind = (2 * pi - startPose(3)) / thetares + 1;
    thetafind = (2 * pi - goalPose(3)) / thetares + 1;
    xind = goalPose(1) / gridres + 1;
    yind = -goalPose(2) / gridres + 1;
elseif goalPose(1) < 0 && goalPose(2) >= 0 % 第二象限
    theta0ind = wrapTo2Pimy(pi - startPose(3)) / thetares + 1;
    thetafind = wrapTo2Pimy(pi - goalPose(3)) / thetares + 1;
    xind = -goalPose(1) / gridres + 1;
    yind = goalPose(2) / gridres + 1;
elseif goalPose(1) < 0 && goalPose(2) < 0 % 第三象限
    theta0ind = wrapTo2Pimy(startPose(3) - pi) / thetares + 1;
    thetafind = wrapTo2Pimy(goalPose(3) - pi) / thetares + 1;
    xind = -goalPose(1) / gridres + 1;
    yind = -goalPose(2) / gridres + 1;
end

theta0ind = round(theta0ind); % 小数化整数，避免除以pi时的小数误差
thetafind = round(thetafind);

if theta0ind == thetadisNum + 1, theta0ind = 1; end
if thetafind == thetadisNum + 1, thetafind = 1; end

if theta0ind > thetadisNum / 4 + 1 || xind > size(dataBase.database, 1) || yind > size(dataBase.database, 2)
    motionPrimitive = [];
    return;
end

if ifconsthetaf
    ind = (theta0ind - 1) * thetadisNum + thetafind;
else
    ind = theta0ind;
end

motionPrimitive = dataBase.database{xind, yind}(ind);
if isinf(motionPrimitive.cost)
    motionPrimitive = [];
    return
end

if goalPose(1) >= 0 && goalPose(2) < 0 % 第四象限
    motionPrimitive.path(:, 2) = -motionPrimitive.path(:, 2);
    motionPrimitive.theta0 = 2 * pi - motionPrimitive.theta0;
    motionPrimitive.thetaf = 2 * pi - motionPrimitive.thetaf;
elseif goalPose(1) < 0 && goalPose(2) >= 0 % 第二象限
    motionPrimitive.path(:, 1) = -motionPrimitive.path(:, 1);
    motionPrimitive.theta0 = pi - motionPrimitive.theta0;
    motionPrimitive.thetaf = pi - motionPrimitive.thetaf;
elseif goalPose(1) < 0 && goalPose(2) < 0 % 第三象限
    motionPrimitive.path(:, 1) = -motionPrimitive.path(:, 1);
    motionPrimitive.path(:, 2) = -motionPrimitive.path(:, 2);
    motionPrimitive.theta0 = motionPrimitive.theta0 - pi;
    motionPrimitive.thetaf = motionPrimitive.thetaf - pi;
end

motionPrimitive.theta0 = wrapTo2Pimy(motionPrimitive.theta0);
motionPrimitive.thetaf = wrapTo2Pimy(motionPrimitive.thetaf);

end

function angle = wrapTo2Pimy(angle)
% 重载matlab的wrapTo2Pi函数，自带的太繁琐

if angle < 0
    angle = angle + 2 * pi;
elseif angle > 2 * pi
    angle = angle - 2 * pi;
end

end


