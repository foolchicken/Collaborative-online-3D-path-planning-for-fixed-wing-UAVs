function [pathXYH, pathLLH, allnodeList, algoParam] = QuickDubinsRRT3dim(algoParam, dynamicCons, mpdata, mpdataFTA, mapdir)
% 3维dubinsRRT路径规划，输入算法参数、运动学约束、两个运动基元数据库

load(mpdata)
load(mpdataFTA)

if mpdataFTA.gridres ~= mpdata.gridres || mpdataFTA.speed ~= mpdata.speed
    error('数据库分辨率或速度不同！')
end
disp('运动基元数据库加载成功！')

generateObs; % 生成障碍物

% dynamicCons.rollmin = -deg2rad(30); % 滚转角
% dynamicCons.rollmax = deg2rad(30);
% dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
% dynamicCons.pitchmax = deg2rad(30);
% dynamicCons.maxh = 1000; % 最大飞行高度

% algoParam.maxIter = 500; % 最大扩展次数，即树的最大容量
% algoParam.start = [109.2, 34.61, 600, deg2rad(90), deg2rad(0)]; % 经纬高、航向角（和x轴夹角）、俯仰角
% algoParam.goal = [108.9, 34.911, 1000, deg2rad(200), deg2rad(0)];
% algoParam.checkStep = 1;
% algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
% algoParam.dsafe = 100; % 最小离地高度

dynamicCons.rollmax = mpdata.dynamicCons.rollmax; % 滚转角直接集成到数据库中
algoParam.dynamicCons = dynamicCons;
algoParam.resolutionMP = mpdata.gridres; % 运动基元网格分辨率
algoParam.speed = mpdata.speed; % 飞行速度
[algoParam.highData, algoParam.mapRef] = readgeoraster(mapdir); % 读取高程地图
algoParam.highData = double(algoParam.highData); % 统一使用double计算
algoParam.obs = Obslist;
for i = 1 : numel(Obslist)
    algoParam.obs(i) = algoParam.obs(i).LL2XY(algoParam.start(1 : 2)); % 障碍物坐标转换
end

disp(['起终点水平直线距离为 ' num2str(distance(algoParam.start(2), algoParam.start(1), algoParam.goal(2), algoParam.goal(1), wgs84Ellipsoid) / 1000) ' km'])
disp('高程地图加载成功！')
disp('地图范围：')
disp(['经度 ' num2str(algoParam.mapRef.LongitudeLimits(1)) '--' num2str(algoParam.mapRef.LongitudeLimits(2))]);
disp(['纬度 ' num2str(algoParam.mapRef.LatitudeLimits(1)) '--' num2str(algoParam.mapRef.LatitudeLimits(2))]);

minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));
disp(['无人机转弯半径为 ' num2str(minR)]);
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点

%% 判断起终点是否可行
if ~PointCheck([0 0 algoParam.start(3)], algoParam)
    error('起点不在地图范围内或者位于障碍物内！');
end
if ~PointCheck(goalxyh, algoParam)
    error('终点不在地图范围内或者位于障碍物内！');
end

n = size(mpdata.database, 1);
range = (n - 1) * mpdata.gridres; % 运动基元数据库范围
step = range * 0.75; % 搜索步长
deltas = mpdata.gridres * 1.5; % 剪枝邻域范围
disp(['运动基元栅格分辨率 ' num2str(mpdata.gridres) ' m'])
disp(['运动基元数据库范围 ' num2str(range) ' m'])

generateNodeStruct; % 生成节点数据结构
RRTree = repmat(StructNode, algoParam.maxIter, 1);

initNode = StructNode;
initNode.state = [0 0 algoParam.start(3 : 5)];
initNode.pos = algoParam.start(1 : 3);
initNode.path = [0 0 algoParam.start(3)]; % 路径为xyh坐标
initNode.cost = 0;
initNode.dh = 0;
initNode.ind = 1;
initNode.parent = -1;
initNode.h = norm(initNode.path - goalxyh(1 : 3)); % 启发值使用欧氏距离
initNode.ifact = true;
[I, J] = geographicToDiscrete(algoParam.mapRef, algoParam.start(2), algoParam.start(1));
initNode.hterrain = algoParam.highData(I, J);
initNode = ObsRangeCheck(initNode, algoParam.obs, range);
RRTree(1) = initNode;
allnodeList = [];

%% 进入迭代
bestLength = inf;
lastbestLength = inf;
a = inf;
b = 0;
c = 0;
iter = 1;
iffirstsample = true;
failedAttempts = 0;
count = 0;
goalind = -1;
newNode = [];
tic
while iter < algoParam.maxIter && failedAttempts < algoParam.maxFailedAttempts
    %% 选择要扩展的节点
    if ~isempty(newNode) && newNode.h < RRTree(newNode.parent).h && isinf(bestLength)
        % 基于上一次节点继续扩展
        nearestNode = newNode;
        sample = goalxyh(1 : 3);
        mindist = norm(newNode.state(1 : 3) - goalxyh(1 : 3));
        eucdist = GetDist2Tree(sample(1 : 3), RRTree(1 : iter), algoParam); % 采样点到RRT树的欧式距离
    else
        % 重新采样
        if iffirstsample
            sample = goalxyh;
        else
            sample = ChooseSample(algoParam, a, b, c); % 选择采样点
        end
        iffirstsample = false;
        if ~PointCheck(sample, algoParam)
            failedAttempts = failedAttempts + 1;
            newNode = [];
            continue
        end
        eucdist = GetDist2Tree(sample(1 : 3), RRTree(1 : iter), algoParam); % 采样点到RRT树的欧式距离
        [mindist, index] = min(eucdist);
        if mindist < 2 % 距离过近，重新采样
            failedAttempts = failedAttempts + 1;
            continue;
        end
        nearestNode = RRTree(index);
    end

    if mindist > step % 如果采样点超出最大步长
        dirxy = atan2(sample(2) - nearestNode.path(end, 2), sample(1) - nearestNode.path(end, 1));
        dirz = atan2(sample(3) - nearestNode.path(end, 3), norm(sample(1 : 2) - nearestNode.path(end, 1 : 2)));
        stepxy = step * cos(dirz);
        stepz = step * sin(dirz);
        sample(1 : 2) = nearestNode.path(end, 1 : 2) + stepxy * [cos(dirxy)  sin(dirxy)]; % 从最近点出发朝采样点前进step长度
        sample(3) = nearestNode.path(end, 3) + stepz;
        sample = NormalizedGrid(sample, algoParam); % 栅格化
        if ~PointCheck(sample, algoParam)
            failedAttempts = failedAttempts + 1;
            newNode = [];
            continue
        end
        eucdist = GetDist2Tree(sample(1 : 3), RRTree(1 : iter), algoParam);
    end


    nearNodes = RRTree(eucdist <= range); % 这样可以减少一次遍历，更快
    %% 扩展RRT树
    if isempty(nearNodes) % 邻接点为空重新采样
        failedAttempts = failedAttempts + 1;
        continue;
    end
    if isequal(sample, goalxyh(1 : 3)) % 采样点为终点时直接使用dubins计算
        newNode = Steer2Goal(nearNodes, algoParam);
    else  % 采样点非目标点时扩展使用终端角自由数据库
        newNode = Steer2Sample(sample, nearNodes, mpdataFTA, algoParam);
    end
    if isempty(newNode) % 扩展节点失败重新采样
        failedAttempts = failedAttempts + 1;
        continue;
    else % 扩展成功计算采样点海拔
        point = XY2LL(sample, algoParam.start); % 再判断是否和地形碰撞
        I = round((algoParam.mapRef.LatitudeLimits(2) - point(2)) / algoParam.mapRef.CellExtentInLatitude) + 1;
        J = round((point(1) - algoParam.mapRef.LongitudeLimits(1)) / algoParam.mapRef.CellExtentInLongitude) + 1;
        newNode.hterrain = algoParam.highData(I, J);
    end
    %newNode.obs = [1];
    newNode = ObsRangeCheck(newNode, algoParam.obs, range);
    failedAttempts = 0;
    iter = iter + 1;
    newNode.ind = iter;
    RRTree(iter) = newNode;
    %% 重新布线
    queue = zeros(int32(algoParam.maxIter / 5), 1);
    if ~isequal(sample, goalxyh(1 : 3)) % 只有采样点不是终点时才重布线
        algoParamt = algoParam;
        algoParamt.obs = algoParamt.obs(newNode.obs);
        for i = 1 : size(nearNodes, 1)
            if nearNodes(i).ind == 1 || nearNodes(i).ind == newNode.parent || nearNodes(i).parent == newNode.parent % 邻居节点是起点和父节点时跳过
                continue;
            end
            if isequal(nearNodes(i).state(1 : 2), goalxyh(1 : 2)) % 邻节点为终点时直接计算dubins
                [pathXYH, length, dubinsParam] = Get3dimDubinsPath(newNode.state, goalxyh, algoParam.speed, algoParam.dynamicCons);
                newcost = newNode.cost + length;
            else
                mp = GetMotionPrimitiveWithHigh(mpdata, newNode.state, nearNodes(i).state, algoParam.dynamicCons, true); % 以新节点为起点，到达邻节点的运动基元，此时考虑终端角约束
                newcost = newNode.cost + mp.cost;
                if(~isinf(mp.cost)), pathXYH = mp.path; end
            end

            if ~isinf(newcost) && PathCollisionCheck(pathXYH, algoParamt)
                if newcost < nearNodes(i).cost
                    RRTree(nearNodes(i).ind).parent = newNode.ind; % 更新父节点索引
                    RRTree(nearNodes(i).ind).path = pathXYH;
                    count = count + 1;
                    % 要后驱改变nearNodes的所有子节点适应度
                    deltaCost = newcost - RRTree(nearNodes(i).ind).cost;
                    deltah = diff(pathXYH(:, 3));
                    newdh = newNode.dh + sum(abs(deltah)) + sum(abs(diff(deltah))); % 一阶差分和二阶差分绝对值之和
                    deltadh = newdh - RRTree(nearNodes(i).ind).dh;
                    fun = @(node) ChangeNodeCostdh(node, deltaCost, deltadh);
                    RRTree = TraversalRRTree(RRTree, nearNodes(i).ind, fun);
                end
            end
            % 剪枝操作
            vecnear2new = nearNodes(i).state(1 : 3) - newNode.state(1 : 3); % 邻节点指向新节点的向量
            temp = [cos(newNode.state(4)), sin(newNode.state(4)); -sin(newNode.state(4)), cos(newNode.state(4))] * vecnear2new(1 : 2)'; % 顺时针旋转theta
            vecnear2new(1 : 2) = temp';
            if vecnear2new(1) ^ 2 / (2 * deltas) ^ 2 + vecnear2new(2) ^ 2 / (deltas) ^ 2 + vecnear2new(3) ^ 2 / (0.5 * deltas) ^ 2 <= 1
                if abs(RRTree(nearNodes(i).ind).cost - newNode.cost) < 1.0 * norm(nearNodes(i).path(end, :) - newNode.path(end, :))
                    RRTree(nearNodes(i).ind).ifact = false;
                end
            end
        end
    end
    if isequal(goalxyh(1 : 3), sample) % 第一次找到终点
        goalind = iter; % 找到终点时的迭代次数
        disp(['在第 ' num2str(goalind) ' 次采样找到可行路径，长度为 ' num2str(newNode.cost / 1000) ' km'])
        toc
    end
    if goalind > 0 && RRTree(goalind).cost < bestLength % 找到更好的路径
        bestLength = RRTree(goalind).cost;
        a = bestLength / 2;
        b = sqrt(a ^ 2 - initNode.h ^ 2 / 4);
        c = 0.5 * sqrt(RRTree(goalind).dh ^ 2 - (algoParam.goal(3) - algoParam.start(3)) ^ 2);
        %RRTreecpoy = RRTree;
        [nodeList, RRTree(1 : iter)] = PathNodeOptimize(RRTree(1 : iter), goalind, algoParam);
        allnodeList = [allnodeList; {nodeList}];
        disp(['在第 ' num2str(iter) ' 次采样找到更佳路径， 新长度为 ' num2str(bestLength / 1000) ' km'])
        if iter > 200 && (lastbestLength - bestLength) / lastbestLength * 100 < algoParam.accu
            break;
        end
    end
    lastbestLength = bestLength;
end
toc

disp(['重布线 ' num2str(count) ' 次'])
[pathXYH, nodeList] = FindWayBack(RRTree(1 : iter), algoParam);

pathLLH = zeros(size(pathXYH));
for i = 1 : size(pathLLH, 1)
    pathLLH(i, :) = XY2LL(pathXYH(i, :), algoParam.start);
end

prunecount = 0;
for i = 1 : iter
    if ~RRTree(i).ifact
        prunecount = prunecount + 1;
    end
end
disp(['剪切 ' num2str(prunecount) ' 次'])


end


