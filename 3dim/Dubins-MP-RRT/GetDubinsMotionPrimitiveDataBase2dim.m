function mpDataBase = GetDubinsMotionPrimitiveDataBase2dim(speed, girdres, thetadisNum, databasePointNum, dynamicCons, savename)
% 计算dubins运动基元二维数据库，并行计算版本加速计算

thetadisNum = max(thetadisNum, 4);
n = sqrt(databasePointNum);

mpDataBase = cell(n); % 运动基元数据库
for i = 1 : n
    for j = 1 : n
        if i == 1 && j == 1, continue; end
        mpDataBase{i, j} = singleloop(i, j, speed, girdres, thetadisNum, dynamicCons);
    end
end
if nargin == 6
    datainfo = whos('mpDataBase');
    if datainfo.bytes / 1024 ^ 2 > 100
        save([savename '.mat'], 'mpDataBase', '-v7.3'); % 数据太大，使用7.3版本格式保存
    else
        save([savename '.mat'], 'mpDataBase');
    end
end

end


function motionPrimitives = singleloop(i, j, speed, girdres, thetadisNum, dynamicCons)

minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));
motionPrimitive.theta0 = [];
motionPrimitive.thetaf = [];
motionPrimitive.cost = inf;
motionPrimitive.path = [];

motionPrimitives = repmat(motionPrimitive, (thetadisNum / 4 + 1) * thetadisNum, 1);

iter = 1;
point = [(i - 1) * girdres, (j - 1) * girdres];
for m = 0 : thetadisNum / 4
    startPose = [0 0 m * 2 * pi / thetadisNum];
    for k = 0 : thetadisNum - 1
        goalPose = [point k * 2 * pi / thetadisNum];

        motionPrimitive.theta0 = startPose(3);
        motionPrimitive.thetaf = goalPose(3);
        motionPrimitive.cost = inf;
        motionPrimitive.path = [];

        pathseg = dubins_core([startPose(1 : 2) pi / 2 - startPose(3)], [goalPose(1 : 2) pi / 2 - goalPose(3)], minR); % 北向航向角转为方向角
        if pathseg.type == 5 || pathseg.type == 6  % 舍弃三段弧的dubins路径
            motionPrimitives(iter) = motionPrimitive;
            iter = iter + 1;
            continue
        end
        if (pathseg.seg_param(1) + pathseg.seg_param(3)) > pi * 1.5 % 圆弧的圆心角大于270度舍弃
            motionPrimitives(iter) = motionPrimitive;
            iter = iter + 1;
            continue
        end
        length = dubins_length(pathseg);
        motionPrimitive.cost = length;
        nseg = 30; % 路径分割的线段数量
        motionPrimitive.path = dubins_path_sample_many(pathseg, length / nseg);
        if size(motionPrimitive.path, 1) == nseg
            motionPrimitive.path = [motionPrimitive.path; goalPose];
        end

        motionPrimitives(iter) = motionPrimitive;
        iter = iter + 1;

    end
end
end


