% 改进MPRRT，借鉴Accelerating Kinodynamic RRT* Through Dimensionality Reduction论文思路进行降维
% 运动基元为dubins路径
% 扩展新节点时从无终端约束数据库中选择运动基元，重布线时使用有终端约束数据库，连接到终点时直接使用dubins进行计算
addpath('D:\Matlabproject\航迹规划算法')
close all
clear
clc

load("mpdataFTAv10.mat")
load("mpdatav10.mat")
if mpdataFTA.gridres ~= mpdata.gridres || mpdataFTA.speed ~= mpdata.speed
    error('数据库分辨率或速度不同！')
end

I = imread('map2.png');
%Ig = rgb2gray(I); % 将 RGB 图像或颜色图转换为灰度图

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);

%algoParam.map = imbinarize(Ig); % 栅格为1表示可行，为0表示不可行
algoParam.map = I;
algoParam.maxIter = 500; % 最大扩展次数，即树的最大容量
% algoParam.start = [40, 20, deg2rad(90)];
% algoParam.goal = [450, 320, deg2rad(0)];
algoParam.start = [50, 20, deg2rad(90)];
algoParam.goal = [480, 420, deg2rad(90)];
algoParam.resolutionMap = 1; % 栅格地图网格分辨率
algoParam.dynamicCons = dynamicCons;
algoParam.checkStep = 1;
algoParam.dsafe = 2; % 到障碍物的最短距离
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.resolutionMP = mpdata.gridres; % 基元数据库网格分辨率
algoParam.speed = mpdata.speed;

minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));

StructNode.theta0 = []; % 该连接的起始航向角
StructNode.thetaf = [];
StructNode.path = []; % 上一节点到当前接节点的转移路径

StructNode.cost = []; % 从起始节点到当前节点的路径代价
StructNode.ind = []; % 节点自身在树中的索引
StructNode.parent = []; % 父节点在树中的索引

RRTree = repmat(StructNode, algoParam.maxIter, 1);
initNode = StructNode;
initNode.thetaf = algoParam.start(3);
initNode.path = algoParam.start(1 : 2);
initNode.cost = 0;
initNode.ind = 1;
initNode.parent = -1;
RRTree(1) = initNode;
n = size(mpdata.database, 1);
range = (n - 1) * mpdata.gridres; % 运动基元数据库范围
step = range / 2; % 搜索步长

%% 进入迭代
bestLength = inf;
iter = 1;
failedAttempts = 0;

count = 0;

tic
while iter < algoParam.maxIter && failedAttempts < algoParam.maxFailedAttempts
    %% 采样
    sample = ChooseSample(algoParam, bestLength); % 选择采样点

    eucdist = zeros(iter, 1); % 采样点到RRT树的欧式距离
    for i = 1 : iter
        eucdist(i) = norm(RRTree(i).path(end, 1 : 2) - sample(1 : 2));
    end
    [mindist, index] = min(eucdist);
    if mindist < 2 % 距离过近，重新采样
        failedAttempts = failedAttempts + 1;
        continue;
    end

    nearestNode = RRTree(index);
    if mindist > range % 如果采样点超出了基元数据库范围
        dir = atan2(sample(2) - nearestNode.path(end, 2), sample(1) - nearestNode.path(end, 1));
        sample = nearestNode.path(end, 1 : 2) + step * [cos(dir)  sin(dir)]; % 从最近点出发朝采样点前进step长度
        sample = NormalizedGrid(sample, algoParam); % 栅格化
    end

    %% 扩展RRT树
    nearNodes = NearNodes(sample, RRTree(1 : iter), range, algoParam); % 采样点的邻结点
    if isempty(nearNodes) % 邻接点为空重新采样
        failedAttempts = failedAttempts + 1;
        continue;
    end
    if isequal(sample, algoParam.goal(1 : 2)) % 采样点为终点时直接使用dubins计算
        newNode = Steer2Goal(nearNodes, algoParam);
    else  % 采样点非目标点时扩展使用终端角自由数据库
        newNode = Steer2Sample(sample, nearNodes, mpdataFTA, algoParam);
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
    queue = zeros(int32(algoParam.maxIter / 5), 1);
    if ~isequal(sample, algoParam.goal(1 : 2)) % 只有采样点不是终点时才重布线
        for i = 1 : size(nearNodes, 1)
            if nearNodes(i).ind == 1 || nearNodes(i).ind == newNode.parent % 邻居节点是起点和父节点时跳过
                continue;
            end
            if isequal(nearNodes(i).path(end, 1 : 2), algoParam.goal(1 : 2)) % 邻节点为终点时直接计算dubins
                param = dubins_core([newNode.path(end, :) newNode.thetaf], algoParam.goal, minR);
                length = dubins_length(param);
                path = dubins_path_sample_many(param, length / 30);
                path = [path(:, 1 : 2); algoParam.goal(1 : 2)];
                newcost = newNode.cost + length;
            else
                mp = GetMotionPrimitive(mpdata, [newNode.path(end, :) newNode.thetaf], [nearNodes(i).path(end, :) nearNodes(i).thetaf]); % 以新节点为起点，到达邻节点的运动基元
                if isempty(mp)
                    path = [];
                    newcost = inf;
                else
                    path = newNode.path(end, 1 : 2) + mp.path(:, 1 : 2);
                    newcost = newNode.cost + mp.cost;
                end
            end
            if ~isinf(newcost) && PathCollisionCheck(path, algoParam)
                if newcost < nearNodes(i).cost
                    RRTree(nearNodes(i).ind).parent = newNode.ind; % 更新父节点索引
                    RRTree(nearNodes(i).ind).path = path;
                    RRTree(nearNodes(i).ind).theta0 = newNode.thetaf; % 起始航向角为新节点的终止航向角
                    % 要后驱改变nearNodes的所有子节点适应度
                    deltaCost = newcost - RRTree(nearNodes(i).ind).cost;
                    bottom = 0;
                    top = 0;
                    bottom = bottom + 1;
                    queue(bottom) = nearNodes(i).ind;
                    count = count + 1;
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
        end
    end
    if isequal(algoParam.goal(1 : 2), sample) % 第一次找到终点
        bestLength = newNode.cost;
        goalind = iter; % 找到终点时的迭代次数
        disp(['在第 ' num2str(goalind) ' 次采样找到可行路径，长度为 ' num2str(bestLength)])
        toc
    end
    if ~isinf(bestLength) && RRTree(goalind).cost < bestLength % 找到更好的路径
        bestLength = RRTree(goalind).cost;
        %disp(['找到更佳路径， 新长度为 ' num2str(bestLength)])
    end
end
toc

disp(['重布线 ' num2str(count) ' 次'])

path = FindWayBack(RRTree(1 : iter), algoParam);


%% 绘制结果和RRT树
figure
plotGrid(algoParam.map)
hold on
scatter(algoParam.start(1) / algoParam.resolutionMap, ...
    algoParam.start(2) / algoParam.resolutionMap);
scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
    algoParam.goal(2) / algoParam.resolutionMap);
for i = 1 : iter
    plot(RRTree(i).path(:, 1) / algoParam.resolutionMap, ...
        RRTree(i).path(:, 2) / algoParam.resolutionMap, 'LineWidth', 1, 'Color', 'r');
end
hold off
grid on
xlabel('x')
ylabel('y')
title('Dubins-MP-RRT Tree')


figure
plotGrid(algoParam.map)
hold on
scatter(algoParam.start(1) / algoParam.resolutionMap, ...
    algoParam.start(2) / algoParam.resolutionMap);
scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
    algoParam.goal(2) / algoParam.resolutionMap);
plot(path(:, 1) / algoParam.resolutionMap, ...
    path(:, 2) / algoParam.resolutionMap, 'LineWidth', 1.5, 'Color', 'r');
hold off
grid on
axis equal
xlabel('x')
ylabel('y')
title('MP-RRT route plan')







