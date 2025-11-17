function mpDataBaseTFA = GetDubinsMotionPrimitiveDataBase2dimFreeTA(speed, girdres, thetadisNum, databasePointNum, dynamicCons, savename)
% 计算dubins运动基元二维数据库，，终端角自由，并行计算版本加速计算

thetadisNum = max(thetadisNum, 4);
if mod(thetadisNum / 4, 2) == 1
    thetadisNum = (thetadisNum / 4 + 1) * 4; % 必须保证每个象限可以被分为偶数份
end
n = sqrt(databasePointNum);

mpDataBaseTFA = cell(n); % 运动基元数据库
for i = 1 : n
    for j = 1 : n
        if i == 1 && j == 1, continue; end
        mpDataBaseTFA{i, j} = singleloop(i, j, speed, girdres, thetadisNum, dynamicCons);
    end
end
mpdataFTA.database = mpDataBaseTFA;
mpdataFTA.gridres = girdres;
mpdataFTA.speed = speed;
mpdataFTA.thetadisNum = thetadisNum;
mpdataFTA.dynamicCons = dynamicCons;
if nargin == 6
    datainfo = whos('mpdataFTA');
    if datainfo.bytes / 1024 ^ 2 > 2000
        save([savename '.mat'], 'mpdataFTA', '-v7.3'); % 数据太大，使用7.3版本格式保存
    else
        save([savename '.mat'], 'mpdataFTA');
    end
end

end


function motionPrimitives = singleloop(i, j, speed, girdres, thetadisNum, dynamicCons)

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));
motionPrimitive.theta0 = [];
motionPrimitive.thetaf = [];
motionPrimitive.cost = inf;
motionPrimitive.path = [];

motionPrimitives = repmat(motionPrimitive, (thetadisNum / 2 + 1), 1);
dtheta = (2 * pi / thetadisNum);

iter = 1;
point = [(i - 1) * girdres, (j - 1) * girdres];
for m = 0 : thetadisNum / 2
    startPose = [0 0 m * 2 * pi / thetadisNum - pi / 4];

    motionPrimitive.theta0 = startPose(3);
    motionPrimitive.cost = inf;
    motionPrimitive.path = [];

    thetaf = get_terminal_free_goal_heading(startPose, point, minR);
    thetaf = wrapTo2Pimy(round(thetaf / dtheta) * dtheta);
    goalPose = [point thetaf];
    motionPrimitive.thetaf = goalPose(3);

    [pathseg, length] = DubinsPath(startPose, goalPose, minR);
    if pathseg.MotionTypes{2} == 'L' || pathseg.MotionTypes{2} == 'R' % 舍弃三段弧的dubins路径
        motionPrimitives(iter) = motionPrimitive;
        iter = iter + 1;
        continue
    end
    if (pathseg.MotionLengths(1) + pathseg.MotionLengths(3)) > minR * pi * 1.5 % 圆弧的圆心角大于270度舍弃
        motionPrimitives(iter) = motionPrimitive;
        iter = iter + 1;
        continue
    end
    motionPrimitive.cost = length;
    nseg = max(length / minR * 4, 5); % 路径分割的线段数量
    motionPrimitive.path = interpolate(pathseg.obj, linspace(0, pathseg.Length, nseg));

    motionPrimitives(iter) = motionPrimitive;
    iter = iter + 1;

end
end


