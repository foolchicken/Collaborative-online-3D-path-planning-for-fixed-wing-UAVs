% 改进MPRRT，借鉴Accelerating Kinodynamic RRT* Through Dimensionality Reduction论文思路进行降维
% 运动基元为dubins路径
% 扩展新节点时从无终端约束数据库中选择运动基元，重布线时使用有终端约束数据库，连接到终点时直接使用dubins进行计算
% 加速RRT，改进策略如下：
% 1 引入启发函数，加速朝着目标前进
% 2 加入剪枝操作，相似节点不加入操作
% 实践证明剪枝操作在后，当迭代次数较多时可以大大减少邻节点数量，加速计算，加速效果随着迭代次数的增大越来越明显
addpath('D:\Matlabproject\航迹规划算法')
addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')
addpath('D:\Matlabproject\mydubins')
close all
clear
clc

load("mpdataFTAv30.mat")
load("mpdatav30.mat")
if mpdataFTA.gridres ~= mpdata.gridres || mpdataFTA.speed ~= mpdata.speed
    error('数据库分辨率或速度不同！')
end

I = imread('map2.png');
%Ig = rgb2gray(I); % 将 RGB 图像或颜色图转换为灰度图

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);

%algoParam.map = imbinarize(Ig); % 栅格为1表示可行，为0表示不可行
algoParam.map = I;
algoParam.maxIter = 1000; % 最大扩展次数，即树的最大容量
% algoParam.start = [50, 70, deg2rad(0)];
% algoParam.goal = [480, 260, deg2rad(-90)];
algoParam.start = [100*10, 40*10, deg2rad(90)];
algoParam.goal = [476*10, 420*10, deg2rad(0)]; 
algoParam.resolutionMap = 10; % 栅格地图网格分辨率
algoParam.dynamicCons = dynamicCons;
algoParam.checkStep = 1;
algoParam.dsafe = 0; % 到障碍物的最短距离
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.resolutionMP = mpdata.gridres; % 基元数据库网格分辨率
algoParam.speed = mpdata.speed;
%algoParam.esdf = bwdist(~algoParam.map, 'euclidean') * algoParam.resolutionMap; % 欧式距离场

minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));

generateNodeStruct;
nbench = 1; % 测试次数

benchRecordMPRRT = [];
for b = 1 : nbench
    runtime = zeros(algoParam.maxIter, 1);
    cost = inf * ones(algoParam.maxIter, 1);
    RRTree = repmat(StructNode, algoParam.maxIter, 1);
    initNode = StructNode;
    initNode.thetaf = algoParam.start(3);
    initNode.pos = algoParam.start(1 : 2);
    initNode.path = algoParam.start;
    initNode.cost = 0;
    initNode.ind = 1;
    initNode.parent = -1;
    initNode.h = norm(initNode.pos - algoParam.goal(1 : 2)); % 启发值使用欧氏距离
    initNode.ifact = true;
    RRTree(1) = initNode;
    n = size(mpdata.database, 1);
    range = (n - 1) * mpdata.gridres; % 运动基元数据库范围
    step = range / 2; % 搜索步长
    deltas = mpdata.gridres * 0; % 剪枝邻域范围

    %% 进入迭代
    bestLength = inf;
    iter = 1;
    iffirstsample = true;
    failedAttempts = 0;
    count = 0;
    kfaild = 0;
    newNode = [];

    tic
    while iter <= algoParam.maxIter && failedAttempts < algoParam.maxFailedAttempts
        % 选择要扩展的节点
        if ~isempty(newNode) && newNode.h < RRTree(newNode.parent).h && isinf(bestLength)
            % 基于上一次节点继续扩展
            nearestNode = newNode;
            sample = algoParam.goal(1 : 2);
            mindist = norm(newNode.pos - algoParam.goal(1 : 2));
            eucdist = GetDist2Tree(sample, RRTree(1 : iter));
        else
            %重新采样
            if iffirstsample
                sample = algoParam.goal(1 : 2);
            else
                sample = ChooseSample(algoParam, bestLength); % 选择采样点
            end
            iffirstsample = false;
            if ~PointCheck(sample, algoParam.map, algoParam.resolutionMap)
                failedAttempts = failedAttempts + 1;
                continue;
            end

            eucdist = GetDist2Tree(sample, RRTree(1 : iter)); % 采样点到RRT树的欧式距离
            [mindist, index] = min(eucdist);
            if mindist < 1 % 距离过近，重新采样
                failedAttempts = failedAttempts + 1;
                continue;
            end
            nearestNode = RRTree(index);
        end

        if mindist > step % 如果采样点超出了基元数据库范围
            dir = atan2(sample(2) - nearestNode.pos(2), sample(1) - nearestNode.pos(1));
            sample = nearestNode.pos + step * [cos(dir)  sin(dir)]; % 从最近点出发朝采样点前进step长度
            sample = NormalizedGrid(sample, algoParam); % 栅格化
            if ~PointCheck(sample, algoParam.map, algoParam.resolutionMap)
                failedAttempts = failedAttempts + 1;
                newNode = [];
                continue;
            end
            eucdist = GetDist2Tree(sample, RRTree(1 : iter));
        end

        nearNodes = RRTree(eucdist <= range); % 采样点的邻结点
        %% 扩展RRT树
        if isempty(nearNodes) % 邻接点为空重新采样
            failedAttempts = failedAttempts + 1;
            continue;
        end
        if isequal(sample, algoParam.goal(1 : 2)) % 采样点为终点时直接使用dubins计算
            newNode = Steer2Goal(nearNodes, algoParam);
        else  % 采样点非目标点时扩展使用终端角自由数据库
            [newNode, errornum] = Steer2Sample(sample, nearNodes, mpdataFTA, algoParam);
        end
        if isempty(newNode) % 扩展节点失败重新采样
            failedAttempts = failedAttempts + 1;
            continue;
        end
        failedAttempts = 0;
        iter = iter + 1;
        newNode.ind = iter;
        RRTree(iter) = newNode;

        %% 重新布线
        queue = zeros(int32(algoParam.maxIter / 10), 1);
        if ~isequal(sample, algoParam.goal(1 : 2)) % 只有采样点不是终点时才重布线
            for i = 1 : size(nearNodes, 1)
                if nearNodes(i).ind == 1 || nearNodes(i).ind == newNode.parent || nearNodes(i).parent == newNode.parent % 邻居节点是起点和父节点时跳过
                    continue;
                end

                if isequal(nearNodes(i).pos, algoParam.goal(1 : 2)) % 邻节点为终点时直接计算dubins
                    param = dubins_core([newNode.pos newNode.thetaf], algoParam.goal, minR);
                    length = dubins_length(param);
                    nseg = max(length / minR * 4, 5); % 路径分割的线段数量
                    path = dubins_path_sample_many(param, length / nseg);
                    path = path(:, 1 : 2);
                    %path = [path; algoParam.goal];
                    newcost = newNode.cost + length;
                else
                    mp = GetMotionPrimitive(mpdata, [newNode.pos, newNode.thetaf], [nearNodes(i).pos, nearNodes(i).thetaf]); % 以新节点为起点，到达邻节点的运动基元
                    if isempty(mp)
                        path = [];
                        newcost = inf;
                    else
                        path = newNode.pos + mp.path(:, 1 : 2);
                        newcost = newNode.cost + mp.cost;
                    end
                end
                if ~isinf(newcost) && PathCollisionCheck(path, algoParam)
                    if newcost < nearNodes(i).cost
                        count = count + 1;
                        RRTree(nearNodes(i).ind).parent = newNode.ind; % 更新父节点索引
                        RRTree(nearNodes(i).ind).path = path;
                        RRTree(nearNodes(i).ind).theta0 = newNode.thetaf; % 起始航向角为新节点的终止航向角

                        % 要后驱改变nearNodes的所有子节点适应度
                        deltaCost = newcost - RRTree(nearNodes(i).ind).cost;
                        bottom = 0;
                        top = 0;
                        bottom = bottom + 1;
                        queue(bottom) = nearNodes(i).ind;

                        while top < bottom % 巧妙地遍历树

                            top = top + 1;
                            cur = queue(top);
                            RRTree(cur).cost = RRTree(cur).cost + deltaCost;
                            childs = [];
                            for j = 1 : iter
                                if RRTree(j).parent == cur
                                    childs = [j, childs];
                                end
                            end
                            for k_ind = 1 : numel(childs)
                                bottom = bottom + 1;
                                queue(bottom) = childs(k_ind);
                            end
                        end
                    end
                end
                % 剪枝操作
                vecnear2new = nearNodes(i).pos - newNode.pos; % 邻节点指向新节点的向量
                vecnear2new = [ cos(newNode.thetaf), sin(newNode.thetaf); -sin(newNode.thetaf), cos(newNode.thetaf)] * vecnear2new'; % 顺时针旋转theta
                if vecnear2new(1) ^ 2 / (2 * deltas) ^ 2 + vecnear2new(2) ^ 2 / (deltas) ^ 2 <= 1
                    %if abs(nearNodes(i).pos(1) - newNode.pos(1)) < deltas && abs(nearNodes(i).pos(2) - newNode.pos(2)) < deltas
                    if abs(RRTree(nearNodes(i).ind).cost - newNode.cost) < 1 * norm(nearNodes(i).pos - newNode.pos) %|| nearNodes(i).h > newNode.h
                        RRTree(nearNodes(i).ind).ifact = false;
                    end
                end
            end
        end
        if isequal(algoParam.goal(1 : 2), sample) % 第一次找到终点
            bestLength = newNode.cost;
            goalind = iter; % 找到终点时的迭代次数
            %disp(['在第 ' num2str(goalind) ' 次采样找到可行路径，长度为 ' num2str(bestLength)])
        end
        if ~isinf(bestLength) && RRTree(goalind).cost < bestLength % 找到更好的路径
            bestLength = RRTree(goalind).cost;
            %disp(['找到更佳路径， 新长度为 ' num2str(bestLength)])
        end
        runtime(iter - 1) = toc;
        cost(iter - 1) = bestLength;
    end
    disp(['最终长度为 ' num2str(bestLength)])
    benchRecordMPRRT(b).runtime = runtime;
    benchRecordMPRRT(b).cost = cost;
end
%save('benchRecordMPRRT.mat', 'benchRecordMPRRT', '-v7.3')

% disp(['重布线 ' num2str(count) ' 次'])
% path = FindWayBack(RRTree(1 : iter), algoParam);
% pathlen = sum(vecnorm(diff(path), 2, 2));
% disp(['路径长度为 ' num2str(pathlen)])
% 
%% 绘制结果和RRT树
figure
plotGrid(algoParam.map)
hold on
scatter(algoParam.start(1) / algoParam.resolutionMap, ...
    algoParam.start(2) / algoParam.resolutionMap);
scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
    algoParam.goal(2) / algoParam.resolutionMap);
for i = 1 : iter
    l1 = plot(RRTree(i).path(:, 1) / algoParam.resolutionMap, ...
        RRTree(i).path(:, 2) / algoParam.resolutionMap, 'LineWidth', 1, 'Color', 'blue');
    l2 = scatter(RRTree(i).path(1, 1) / algoParam.resolutionMap, RRTree(i).path(1, 2) / algoParam.resolutionMap, 'black');
end
hold off
grid on
xlabel('x/m')
ylabel('y/m')
legend([l1 l2],{'edge', 'node'})
title('Dubins-MP-RRT Tree')
% 
% 
% figure
% plotGrid(algoParam.map)
% hold on
% scatter(algoParam.start(1) / algoParam.resolutionMap, ...
%     algoParam.start(2) / algoParam.resolutionMap);
% scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
%     algoParam.goal(2) / algoParam.resolutionMap);
% plot(path(:, 1) / algoParam.resolutionMap, ...
%     path(:, 2) / algoParam.resolutionMap, 'LineWidth', 1.5, 'Color', 'r');
% hold off
% grid on
% axis equal
% xlabel('x/m')
% ylabel('y/m')
% title('MP-RRT route plan')


