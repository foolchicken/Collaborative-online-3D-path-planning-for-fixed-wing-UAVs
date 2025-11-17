function motionPrimitive = GetMotionPrimitive(dataBase, startPose, goalPose)
% 从数据库中提取运动基元

gridres = dataBase.gridres;
thetadisNum = dataBase.thetadisNum;
thetares = 2 * pi / thetadisNum;
goalPose(1) = goalPose(1) - startPose(1); % 先平移到原点
goalPose(2) = goalPose(2) - startPose(2);

motionPrimitive = [];
if goalPose(1) == 0 && goalPose(2) == 0
    return
end
xind = abs(goalPose(1)) / gridres + 1;
yind = abs(goalPose(2)) / gridres + 1;
if xind > size(dataBase.database, 1) || yind > size(dataBase.database, 2)
    return;
end

theta0 = startPose(3); % 起点航向角
thetaf = goalPose(3); % 终点航向角
if goalPose(1) < 0 && goalPose(2) >= 0 % 第二象限
    theta0 = pi - theta0;
    thetaf = pi - thetaf;
elseif goalPose(1) < 0 && goalPose(2) < 0 % 第三象限
    theta0 = theta0 - pi;
    thetaf = thetaf - pi;
elseif goalPose(1) >= 0 && goalPose(2) < 0 % 第四象限
    theta0 = 2 * pi - theta0;
    thetaf = 2 * pi - thetaf;
end

% 角度定义在[-pi,pi]
if theta0 < -pi
    theta0 = theta0 + 2 * pi;
elseif theta0 > pi
    theta0 = theta0 - 2 * pi;
end
% 角度定义在[0,2pi)
if thetaf < 0
    thetaf = thetaf + 2 * pi;
elseif thetaf >= 2*pi
    thetaf = thetaf - 2 * pi;
end

theta0ind = round((theta0 + pi / 4) / thetares + 1); % 小数化整数，避免除以pi时的小数误差
thetafind = round(thetaf / thetares + 1);

if thetafind == thetadisNum + 1
    thetafind = 1; 
end

if theta0ind < 1 || theta0ind > thetadisNum / 2 + 1
    motionPrimitive = [];
    return;
end

ind = (theta0ind - 1) * thetadisNum + thetafind;

motionPrimitive = dataBase.database{xind, yind}(ind);
if isinf(motionPrimitive.cost)
    motionPrimitive = [];
    return
end

if goalPose(1) >= 0 && goalPose(2) < 0 % 第四象限
    motionPrimitive.path(:, 2) = -motionPrimitive.path(:, 2);
elseif goalPose(1) < 0 && goalPose(2) >= 0 % 第二象限
    motionPrimitive.path(:, 1) = -motionPrimitive.path(:, 1);
elseif goalPose(1) < 0 && goalPose(2) < 0 % 第三象限
    motionPrimitive.path(:, 1) = -motionPrimitive.path(:, 1);
    motionPrimitive.path(:, 2) = -motionPrimitive.path(:, 2);
end

motionPrimitive.theta0 = theta0;
motionPrimitive.thetaf = thetaf;

end



