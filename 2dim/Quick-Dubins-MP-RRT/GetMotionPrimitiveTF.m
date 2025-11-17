function motionPrimitive = GetMotionPrimitiveTF(dataBase, startPose, goalPose)
% 从无终端约束数据库中提取运动基元

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
if goalPose(1) < 0 && goalPose(2) >= 0 % 第二象限
    theta0 = pi - theta0;
elseif goalPose(1) < 0 && goalPose(2) < 0 % 第三象限
    theta0 = theta0 - pi;
elseif goalPose(1) >= 0 && goalPose(2) < 0 % 第四象限
    theta0 = 2 * pi - theta0;
end

% 角度定义在[-pi,pi]
if theta0 < -pi
    theta0 = theta0 + 2 * pi;
elseif theta0 > pi
    theta0 = theta0 - 2 * pi;
end

theta0ind = round((theta0 + pi / 4) / thetares + 1); % 小数化整数，避免除以pi时的小数误差

if theta0ind < 1 || theta0ind > thetadisNum / 2 + 1
    return
    %warning('角度转化错误')
end

motionPrimitive = dataBase.database{xind, yind}(theta0ind);
if isinf(motionPrimitive.cost)
    motionPrimitive = [];
    return
end

motionPrimitive.theta0 = startPose(3);
if goalPose(1) >= 0 && goalPose(2) < 0 % 第四象限
    motionPrimitive.path(:, 2) = -motionPrimitive.path(:, 2);
    motionPrimitive.thetaf = 2 * pi - motionPrimitive.thetaf;
elseif goalPose(1) < 0 && goalPose(2) >= 0 % 第二象限
    motionPrimitive.path(:, 1) = -motionPrimitive.path(:, 1);
    motionPrimitive.thetaf = pi - motionPrimitive.thetaf;
elseif goalPose(1) < 0 && goalPose(2) < 0 % 第三象限
    motionPrimitive.path(:, 1) = -motionPrimitive.path(:, 1);
    motionPrimitive.path(:, 2) = -motionPrimitive.path(:, 2);
    motionPrimitive.thetaf = motionPrimitive.thetaf - pi;
end

if motionPrimitive.thetaf < -pi
    motionPrimitive.thetaf = motionPrimitive.thetaf + 2 * pi;
elseif motionPrimitive.thetaf > pi
    motionPrimitive.thetaf = motionPrimitive.thetaf - 2 * pi;
end

end

% function angle = wrapTo2Pimy(angle)
% % 重载matlab的wrapTo2Pi函数，自带的太繁琐
%
% if angle < 0
%     angle = angle + 2 * pi;
% elseif angle > 2 * pi
%     angle = angle - 2 * pi;
% end
%
% end


