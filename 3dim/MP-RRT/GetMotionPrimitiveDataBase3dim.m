function mpDataBase = GetMotionPrimitiveDataBase3dim(speed, gridresxy, gridresz, yawdisNum, pitchdisNum, databasePointNum, regionCons, dynamicCons, savename)

pitchdisNum = max(pitchdisNum, 3);
yawdisNum = max(yawdisNum, 4);
dynamicCons.vmin = speed;
dynamicCons.vmax = speed;
n = sqrt(databasePointNum);

mpDataBase = cell(n, n, n); % 运动基元数据库
for i = 1 : n
    for j = 1 : n
        if i == 1 && j == 1, continue; end
        for k = 1 : n
            mpDataBase{i, j, k} = singleloop(i, j, k, speed, gridresxy, gridresz, yawdisNum, pitchdisNum, regionCons, dynamicCons);
        end
    end
end
if nargin == 7
    savename = 'mpDataBase';
end
datainfo = whos('mpDataBase');
if datainfo.bytes / 1024 ^ 2 > 100
    save([savename '.mat'], 'mpDataBase', '-v7.3'); % 数据太大，使用7.3版本格式保存
else
    save([savename '.mat'], 'mpDataBase');
end
end

function motionPrimitives = singleloop(i, j, k, speed, gridresxy, gridresz, yawdisNum, pitchdisNum, regionCons, dynamicCons)

motionPrimitive.yaw0 = 0;
motionPrimitive.pitch0 = 0;
motionPrimitive.yawf = 0;
motionPrimitive.pitchf = 0;
motionPrimitive.cost = inf;
motionPrimitive.path = [];

motionPrimitives = repmat(motionPrimitive, (yawdisNum / 4 + 1) * yawdisNum * ceil(pitchdisNum / 2) ^ 2, 1);

iter = 1;
point = [(i - 1) * gridresxy, (j - 1) * gridresxy, (k - 1) * gridresz]; % 目标点坐标
minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));
dyaw = 2 * pi / yawdisNum; % 航向角粒度
dpitch = (dynamicCons.pitchmax - dynamicCons.pitchmin) / (pitchdisNum - 1); % 俯仰角粒度

if point(1) < regionCons(1, 1) || point(1) > regionCons(1, 2) || ...
        point(2) < regionCons(2, 1) || point(2) > regionCons(2, 2) || ...
        point(3) < regionCons(3, 1) || point(3) > regionCons(3, 2)
    error('目标位置不在区域范围内！')
end

for i = 0 : yawdisNum / 4
    for j = 1 : ceil(pitchdisNum / 2)
        startPose = [0 0 0 i * dyaw (j - 1) * dpitch]; % 起点姿态
        for ii = 1 : yawdisNum
            for jj = 1 : pitchdisNum
                goalPose = [point, (ii - 1) * dyaw, dynamicCons.pitchmin + (jj - 1) * dpitch];

                motionPrimitive.yaw0 = startPose(4);
                motionPrimitive.pitch0 = startPose(5);
                motionPrimitive.yawf = goalPose(4);
                motionPrimitive.pitchf = goalPose(5);
                motionPrimitive.cost = inf;
                motionPrimitive.path = [];

                % 通过二维dubins和俯仰角约束先判断是否有解，加快速度
                dubinsParam = dubins_core([startPose(1 : 2) pi / 2 - startPose(4)], [goalPose(1 : 2) pi / 2 - goalPose(4)], minR);
                if dubinsParam.type == 5 || dubinsParam.type == 6 || norm(goalPose(1 : 2) - startPose(1 : 2)) / cos(dynamicCons.pitchmax) < norm(goalPose(1 : 3) - startPose(1 : 3))
                    motionPrimitives(iter) = motionPrimitive;
                    iter = iter + 1;
                    continue
                end

                % 剪枝操作，不必要的基元可以舍弃
                if dubinsParam.seg_param(1) + dubinsParam.seg_param(3) > pi
                    motionPrimitives(iter) = motionPrimitive;
                    iter = iter + 1;
                    continue
                end

                [state, control, cost] = SolveTPBVP3dim(startPose, goalPose, speed, regionCons, dynamicCons);

                if goalPose(4) > pi
                    goalPose(4) = goalPose(4) - 2 * pi;
                    [state1, control1, cost1] = SolveTPBVP3dim(startPose, goalPose, speed, regionCons, dynamicCons);
                    if cost1 < cost
                        cost = cost1;
                        state = state1;
                        control = control1;
                    end
                end
                if isinf(cost)
                    motionPrimitives(iter) = motionPrimitive;
                    iter = iter + 1;
                    continue
                end

                motionPrimitive.cost = cost;
                motionPrimitive.path = DouglasPeucker(state(:, 1 : 3), 10); % 使用抽稀后的线段保存，节约内存
                motionPrimitives(iter) = motionPrimitive;
                iter = iter + 1;
            end
        end
    end
end

end
