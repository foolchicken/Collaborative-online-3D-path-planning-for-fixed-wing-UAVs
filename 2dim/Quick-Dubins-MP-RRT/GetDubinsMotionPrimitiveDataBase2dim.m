function mpDataBase = GetDubinsMotionPrimitiveDataBase2dim(speed, girdres, thetadisNum, databasePointNum, dynamicCons, savename)
% 计算dubins运动基元二维数据库，并行计算版本加速计算

thetadisNum = max(thetadisNum, 4);
if mod(thetadisNum / 4, 2) == 1
    thetadisNum = (thetadisNum / 4 + 1) * 4; % 必须保证每个象限可以被分为偶数份
end
n = sqrt(databasePointNum);

mpDataBase = cell(n); % 运动基元数据库
parfor i = 1 : n
    for j = 1 : n
        if i == 1 && j == 1, continue; end
        mpDataBase{i, j} = singleloop(i, j, speed, girdres, thetadisNum, dynamicCons);
    end
end
mpdata.database = mpDataBase;
mpdata.gridres = girdres;
mpdata.speed = speed;
mpdata.thetadisNum = thetadisNum;
mpdata.dynamicCons = dynamicCons;
if nargin == 6
    datainfo = whos('mpdata');
    if datainfo.bytes / 1024 ^ 2 > 2000
        save([savename '.mat'], 'mpdata', '-v7.3'); % 数据太大，使用7.3版本格式保存
    else
        save([savename '.mat'], 'mpdata');
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
for m = 0 : thetadisNum / 2
    startPose = [0 0 m * 2 * pi / thetadisNum - pi / 4];
    for k = 0 : thetadisNum - 1
        goalPose = [point k * 2 * pi / thetadisNum];

        motionPrimitive.theta0 = startPose(3);
        motionPrimitive.thetaf = wrapTo2Pimy(goalPose(3));
        motionPrimitive.cost = inf;
        motionPrimitive.path = [];

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
        nseg = max(length / minR * 4, 5); % 路径分割的线段数量
        motionPrimitive.cost = length;
        motionPrimitive.path = interpolate(pathseg.obj, linspace(0, pathseg.Length, nseg));

        %         pathseg = dubins_core(startPose, goalPose, minR);
        %         if pathseg.type == 5 || pathseg.type == 6  % 舍弃三段弧的dubins路径
        %             motionPrimitives(iter) = motionPrimitive;
        %             iter = iter + 1;
        %             continue
        %         end
        %         if (pathseg.seg_param(1) + pathseg.seg_param(3)) > pi * 1.5 % 圆弧的圆心角大于270度舍弃
        %             motionPrimitives(iter) = motionPrimitive;
        %             iter = iter + 1;
        %             continue
        %         end
        %         length = dubins_length(pathseg);
        %         motionPrimitive.cost = length;
        %         step = minR/4;
        %         motionPrimitive.path = dubins_path_sample_many(pathseg, step);
        %         if sum(abs(motionPrimitive.path(end, 1 : 2) - goalPose(1 : 2))) > 1e-1
        %             motionPrimitive.path = [motionPrimitive.path; goalPose];
        %         end

        motionPrimitives(iter) = motionPrimitive;
        iter = iter + 1;

    end
end
end


