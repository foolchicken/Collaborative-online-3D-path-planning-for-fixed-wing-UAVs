% 基于2次B样条曲线的RRT*，参考论文：Smooth path planning under maximum curvature constraints
% for autonomous underwater vehicles based on rapidly-exploring random tree star with B-spline curves
% 由于2次样条曲线需要三个控制点计算，所有在扩展节点时新节点的路径不是从最近节点出发
% 而是从最近节点的父节点和最近节点的中点出发，并且终点也不是采样点，而是采样点和最近点的中点
% 虽然计算路径速度比dubinsRRT快，但是由于曲率限制采样效率不高，
% 大部分采样点不符合曲率约束，并且重布线还需要检测子节点的路径，针对此问题可以改进
% 实践证明把连接到最近节点和选择最佳父节点合并效果更好
% 唯一的缺点就是不好实现绕飞

addpath('D:\Matlabproject\航迹规划算法')
addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')
close all
clear
clc
% load("mpdataFTAv10.mat")
% load("mpdatav10.mat")
I = imread('map2.png'); 
%Ig = rgb2gray(I); % 将 RGB 图像或颜色图转换为灰度图

%algoParam.map = imbinarize(Ig); % 栅格为1表示可行，为0表示不可行
algoParam.map = I; 
algoParam.maxIter = 1000; % 最大扩展次数，即树的最大容量
algoParam.start = [120, 25, deg2rad(90)]; %[100, 30, deg2rad(90)];
algoParam.goal = [476, 420, deg2rad(0)]; 
algoParam.maxstep = 50; % 最大步长
algoParam.disTh = 100; % 邻居判断距离
algoParam.resolutionMap = 1; % 栅格地图网格分辨率
algoParam.checkStep = 1; 
algoParam.dsafe = 2; % 到障碍物的最短距离
algoParam.maxFailedAttempts = inf; % 节点最大扩展失败次数
algoParam.r = 17; % dubins转弯半径

%% 判断起终点是否可行
if ~PointCheck(algoParam.start, algoParam.map, algoParam.resolutionMap)
    error('起点不在地图范围内或者位于障碍物内！'); 
end
if ~PointCheck(algoParam.goal, algoParam.map, algoParam.resolutionMap)
    error('终点不在地图范围内或者位于障碍物内！'); 
end

generateNodeStruct; 
RRTree = repmat(StructNode, algoParam.maxIter, 1); 
nstart1 = StructNode; 
nstart1.pos = algoParam.start(1 : 2) + algoParam.maxstep / 2 * [cos(algoParam.start(3) + pi) sin(algoParam.start(3) + pi)]; 
nstart1.cost = 0; 
nstart1.ind = 1; 
nstart1.parent = -1; 
RRTree(1) = nstart1; 

nstart2 = StructNode; 
nstart2.pos = algoParam.start(1 : 2) + algoParam.maxstep / 2 * [cos(algoParam.start(3)) sin(algoParam.start(3))]; 
nstart2.cost = 0; 
nstart2.ind = 2; 
nstart2.parent = 1; 
RRTree(2) = nstart2; 

nend1 = algoParam.goal(1 : 2) + algoParam.maxstep / 2 * [cos(algoParam.goal(3) + pi) sin(algoParam.goal(3) + pi)]; 
nend2 = algoParam.goal(1 : 2) + algoParam.maxstep / 2 * [cos(algoParam.goal(3)) sin(algoParam.goal(3))]; 
%% 进入迭代
iter = 2; 
failedAttempts = 0; 
count = 0; 
goalind = 0; 
bestLength = inf; 
iffindpath = false; % 是否发现路径
kfaild = 0; % 因为曲率不满足的失败次数
ii = 1; 
tic
while iter < algoParam.maxIter && failedAttempts < algoParam.maxFailedAttempts
    %% 采样
    sample = ChooseSample(algoParam, bestLength); % 选择采样点

    eucdist = zeros(iter, 1); % 采样点到RRT树的欧式距离
    eucdist(1) = inf; % 第一个节点只作为计算样条曲线时的父节点使用，不参与最近节点的计算
    for i = 2 : iter
        eucdist(i) = norm(RRTree(i).pos - sample); 
    end
    [mindist, index] = min(eucdist); 
    nearestNode = RRTree(index); 
    if mindist > algoParam.maxstep
        dir = atan2(sample(2) - nearestNode.pos(2), sample(1) - nearestNode.pos(1)); 
        sample = nearestNode.pos + algoParam.maxstep * [cos(dir)  sin(dir)]; % 从最近点出发朝采样点前进step长度
        for i = 2 : iter % 重新计算采样点到RRT树的欧式距离
            eucdist(i) = norm(RRTree(i).pos - sample); % 只计算活跃节点
        end
        [mindist, index] = min(eucdist); 
    end

    if mindist < 1 % 距离过近，重新采样
        failedAttempts = failedAttempts + 1; 
        continue; 
    end
    if  ~PointCheck(sample, algoParam.map, algoParam.resolutionMap)
        kfaild = kfaild + 1; 
        er(ii) = 1; 
        ii = ii + 1; 
        failedAttempts = failedAttempts + 1; 
        continue; 
    end
    %% 扩展RRT树
    %    newNode = Steer2Sample(sample, nearestNode, RRTree, algoParam);
    %     if isinf(newNode.cost)
    %         kfaild = kfaild + 1;
    %     end
    %
    %     if ~PathCollisionCheck(newNode.path, algoParam)
    %         failedAttempts = failedAttempts + 1;
    %         continue;
    %     end

    %% 重新选择父节点
    neighboorIndex = find(eucdist <= algoParam.disTh); % newPoint的邻居节点索引
    newNode.pos = sample; 
    newNode.cost = inf; 
    for i = 1 : numel(neighboorIndex)
        nearpind = RRTree(neighboorIndex(i)).parent; % 邻节点的父节点索引
        [path, length, maxk] = GetBspline2Order(RRTree(nearpind).pos, RRTree(neighboorIndex(i)).pos, sample); 
        if ~isinf(maxk) && maxk <= 1 / algoParam.r && PathCollisionCheck(path, algoParam)
            newcost = RRTree(neighboorIndex(i)).cost + length; % 以neighboor为父节点，从起点到newPoint的代价值c
            if newcost < newNode.cost % 更新父节点和适应度
                nearestNode = RRTree(neighboorIndex(i), :); 
                newNode.cost = newcost; 
                newNode.parent = neighboorIndex(i); 
                newNode.path = path; 
            end
        end
    end
    %     if  ~PointCheck(sample, algoParam.map, algoParam.resolutionMap)
    %         kfaild = kfaild + 1;
    %     end
    if isinf(newNode.cost)
        if isinf(maxk) || maxk > 1 / algoParam.r
            er(ii) = 3; 
        else
            er(ii) = 2; 
        end
        ii = ii + 1; 
        kfaild = kfaild + 1; 
        failedAttempts = failedAttempts + 1; 
        continue; 
    end
    kr(iter) = kfaild; 
    failedAttempts = 0; 
    iter = iter + 1; 
    newNode.ind = iter; 
    RRTree(iter, :) = newNode; % 拓展节点

    %% 重新布线
%     queue = zeros(int32(algoParam.maxIter / 5), 1); 
%     for i = 1 : numel(neighboorIndex)
%         if neighboorIndex(i) == 1 || neighboorIndex(i) == 2 || neighboorIndex(i) == newNode.parent % 邻居节点是起点和父节点时跳过
%             continue
%         end
%         [path, length, maxk] = GetBspline2Order(RRTree(newNode.parent).pos, sample, RRTree(neighboorIndex(i)).pos); % 以新节点为父节点
%         newcost = newNode.cost + length; 
% 
%         if newcost < RRTree(neighboorIndex(i)).cost && maxk <= 1 / algoParam.r && PathCollisionCheck(path, algoParam) % 代价更优&&曲率满足&&无碰撞
% 
%             % 由于重新连接操作还将影响重新连接的节点𝑛i的子节点，所以还要进行额外检测，必须所有子节点路径也可行才重布线
%             ifrewirevalid = true; 
%             if neighboorIndex(i) == goalind % 如果邻节点是endnode1，需要额外检测
%                 [patht, ~, maxkt] = GetBspline2Order(newNode.pos, RRTree(neighboorIndex(i)).pos, nend2); 
%                 if maxkt > 1 / algoParam.r || ~PathCollisionCheck(patht, algoParam)
%                     ifrewirevalid = false; % 如果到终点的路径无效，则不进行重布线
%                     break
%                 end
%             end
%             for j = 1 : iter
%                 if RRTree(j).parent == neighboorIndex(i) % 还会影响被重布线节点的一级子节点
%                     [pathc, lengthc, maxkc] = GetBspline2Order(newNode.pos, RRTree(neighboorIndex(i)).pos, RRTree(j).pos); 
%                     if maxkc > 1 / algoParam.r || ~PathCollisionCheck(pathc, algoParam)
%                         ifrewirevalid = false; % 如果子节点的路径无效，则不进行重布线
%                         break
%                     end
%                 end
%             end
%             if ~ifrewirevalid
%                 continue
%             end
%             count = count + 1; 
%             RRTree(neighboorIndex(i)).parent = newNode.ind; % 更新父节点索引
%             RRTree(neighboorIndex(i)).path = path; % 更新路径
%             RRTree(neighboorIndex(i)).cost = newcost; 
% 
%             % 要后驱改变neighboor的所有子节点适应度（实际受一级子节点影响，修改适应度要复杂的多）
%             for j = 1 : iter
%                 if RRTree(j).parent == neighboorIndex(i) % 还会影响被重布线节点的一级子节点
%                     [pathc, lengthc, maxk] = GetBspline2Order(newNode.pos, RRTree(neighboorIndex(i)).pos, RRTree(j).pos); 
%                     deltaCost = newcost + lengthc - RRTree(j).cost; 
%                     RRTree(j).path = pathc; 
%                     fun = @(node) ChangeNodeCost(node, deltaCost); 
%                     RRTree(1 : iter) = TraversalRRTree(RRTree(1 : iter), j, fun); 
%                 end
%             end
%         end
%     end

    if ~iffindpath % 如果nend1不在RRTree中
        for i = 1 : iter
            if norm(RRTree(i).pos - nend1) < 1e-1
                iffindpath = true; 
            end
        end
        endNode1 = Steer2Goal(newNode, RRTree, algoParam); % 尝试向终点连线
        if ~isinf(endNode1.cost) % 如果路径可行
            iffindpath = true; 
            kr(iter) = kfaild; 
            iter = iter + 1; 
            endNode1.ind = iter; 
            RRTree(iter, :) = endNode1; % 拓展节点

            goalind = iter; % 找到终点时的迭代次数
            [pathend, len2end] = GetBspline2Order(RRTree(endNode1.parent).pos, nend1, nend2); 
%             bestLength = endNode1.cost; 
%             bestLength = bestLength + len2end; 
%             disp(['在第 ' num2str(goalind) ' 次采样找到可行路径，长度为 ' num2str(bestLength)])
        end
    end
%     if ~isinf(bestLength)
%         temp = RRTree(goalind).parent;
%         [pathend, len2end] = GetBspline2Order(RRTree(temp).pos, nend1, nend2);
%         if RRTree(goalind).cost + len2end < bestLength % 找到更好的路径
%             bestLength = RRTree(goalind).cost + len2end;
%             %disp(['找到更佳路径， 新长度为 ' num2str(bestLength)])
%         end
%     end
end
toc

disp(['重布线 ' num2str(count) ' 次'])
solution = FindWayBack(RRTree, algoParam); 
pathlen = sum(vecnorm(diff(solution), 2, 2)); 
disp(['路径长度为 ' num2str(pathlen)])

figure
plotGrid(algoParam.map)
hold on
scatter(algoParam.start(1) / algoParam.resolutionMap, ...
    algoParam.start(2) / algoParam.resolutionMap); 
scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
    algoParam.goal(2) / algoParam.resolutionMap); 
for ii = 3 : iter
    plot(RRTree(ii).path(:, 1) / algoParam.resolutionMap, ...
        RRTree(ii).path(:, 2) / algoParam.resolutionMap, 'LineWidth', 1, 'Color', 'r'); 
end
hold off
grid on
xlabel('x')
ylabel('y')
title('Bspline-RRT Tree')

figure
plotGrid(algoParam.map)
hold on
scatter(algoParam.start(1) / algoParam.resolutionMap, ...
    algoParam.start(2) / algoParam.resolutionMap); 
scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
    algoParam.goal(2) / algoParam.resolutionMap); 
plot(solution(:, 1) / algoParam.resolutionMap, ...
    solution(:, 2) / algoParam.resolutionMap, 'LineWidth', 1.5, 'Color', 'r'); 
% plot(path(:, 1) , path(:, 2))
hold off
grid on
axis equal
xlabel('x')
ylabel('y')
title('Bspline-RRT route plan')
%
% figure
% plot(kr)
%
%
% aa = 0;
% for i = 1 : 1000
%     sample = ChooseSample(algoParam, inf); % 选择采样点
%     if ~PointCheck(sample, algoParam.map, algoParam.resolutionMap)
%         aa = aa + 1;
%     end
% end
sum(er == 3)

function node = ChangeNodeCost(node, deltaCost)
node.cost = node.cost + deltaCost; 
end
