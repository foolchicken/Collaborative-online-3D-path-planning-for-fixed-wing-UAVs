% 改进MPRRT，参考论文：Sampling-based optimal kinodynamic planning with motion primitives
% 借鉴Accelerating Kinodynamic RRT* Through Dimensionality Reduction论文思路进行降维
% 运动基元为dubins路径+高度规划（高度规划使用双线性积分模型求解无约束最优控制）
% 扩展新节点时从无终端约束数据库中选择运动基元，重布线时使用有终端约束数据库，连接到终点时直接使用dubins进行计算
% 加速RRT，改进策略如下：
% 1 引入启发函数，加速朝着目标前进
% 2 加入剪枝操作，相似节点不加入操作
% 3 改进碰撞检测，只对有可能碰撞的障碍物进行检测
% 实践证明剪枝操作在后可以大大减少邻节点数量，加速计算
% 三维RRT主要耗时在碰撞检测，相比与栅格地图碰撞检测时间复杂度大大增加
% 再说一句，matlab的类是真的慢，优化太垃圾了

addpath('D:\Matlabproject\航迹规划算法')
addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')
addpath('D:\Matlabproject\硕士毕业论文\地图处理')
addpath('D:\Matlabproject\硕士毕业论文\3dim')
addpath('D:\Matlabproject\硕士毕业论文\2dim\Quick-Dubins-MP-RRT')
close all
clear
clc

% load("mpdataFTAv90.mat")
% load("mpdatav90.mat")
load("mpdatav60.mat")
load("mpdataFTAv60.mat")
if mpdataFTA.gridres ~= mpdata.gridres || mpdataFTA.speed ~= mpdata.speed
    error('数据库分辨率或速度不同！')
end
disp('运动基元数据库加载成功！')

generateObs; % 生成障碍物

dynamicCons = mpdata.dynamicCons; % 滚转角取基元数据库
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.maxh = 3000; % 最大飞行高度

algoParam.maxIter = 500; % 最大扩展次数，即树的最大容量
% algoParam.start = [109.2, 34.61, 600, deg2rad(90), deg2rad(0)]; % 经纬高、航向角（和x轴夹角）、俯仰角
% algoParam.goal = [108.9, 34.911, 1000, deg2rad(200), deg2rad(0)];
% algoParam.start = [103.665, 39.56, 1700, deg2rad(90), deg2rad(0)]; % 经纬高、航向角（和x轴夹角）、俯仰角
% algoParam.goal = [103.735289, 39.585, 2000, deg2rad(180), deg2rad(0)];
algoParam.start = [108.851497597421	35.1622671226846 1400, deg2rad(0), deg2rad(0)]; % 经纬高、航向角（和x轴夹角）、俯仰角
algoParam.goal = [109.040782330388	35.1429152923380, 1200, deg2rad(90), deg2rad(0)];
algoParam.dynamicCons = dynamicCons;
algoParam.checkStep = 1;
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.resolutionMP = mpdata.gridres; % 运动基元网格分辨率
algoParam.speed = mpdata.speed; % 飞行速度
algoParam.dsafe = 150; % 最小离地高度
[algoParam.highData, algoParam.mapRef] = readgeoraster('D:\MapService\西安北部地图small2.tif'); %('D:\MapService\沙漠地图small.tif D:\MapService\西安高程地图small.tif'); % 读取高程地图
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
step = range / 2; % 搜索步长
deltas = mpdata.gridres * 2; % 剪枝邻域范围
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
        mindist = norm(newNode.path(end, 1 : 3) - goalxyh(1 : 3));
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
        [I, J] = geographicToDiscrete(algoParam.mapRef, point(2), point(1));
        newNode.hterrain = algoParam.highData(I, J);
    end
    %newNode.obs = [];
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
                [pathXYH, length, dubinsParam] = Get3dimDubinsPath(newNode.state, goalxyh, algoParam.speed, algoParam.dynamicCons, true);
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
        %PathNodeOptimize(RRTree(1 : iter), goalind, algoParam);
        RRTreecpoy = RRTree;
        [nodeList, RRTree(1 : iter)] = PathNodeOptimize(RRTree(1 : iter), goalind, algoParam);
        allnodeList = [allnodeList; {nodeList}];
        bestLength = RRTree(goalind).cost;
        a = bestLength / 2;
        b = sqrt(a ^ 2 - initNode.h ^ 2 / 4);
        c = 0.5 * sqrt(RRTree(goalind).dh ^ 2 - (algoParam.goal(3) - algoParam.start(3)) ^ 2);
        disp(['找到更佳路径， 新长度为 ' num2str(bestLength / 1000) ' km'])
    end
end
toc

disp(['重布线 ' num2str(count) ' 次'])
[pathXYH, nodeList] = FindWayBack(RRTree(1 : iter), algoParam);

[pathXYHnew, nodeListnew] = PathHSmooth(nodeList, algoParam);
disp(['高度平滑后的路径长度减少了 ' num2str(nodeList(end).cost - nodeListnew(end).cost) ' m'])
if ~PathCollisionCheck(pathXYHnew, algoParam)
    warning('路径碰撞')
end

path = Get3dimDubinsPath(RRTree(2).state, RRTree(3).state, algoParam.speed, algoParam.dynamicCons, false);
% mp = GetMotionPrimitiveWithHigh(mpdata, nodeList(1).state, nodeList(2).state,  algoParam.dynamicCons, true);
% pathmp = mp.path;
% PathCollisionCheck(path, algoParam)
% figure
% plot3(path(:,1),path(:,2),path(:,3), 'DisplayName', 'dubins')
% hold on
% %plot3(nodeList(2).path(:,1),nodeList(2).path(:,2),nodeList(2).path(:,3), 'DisplayName', 'mp')
% plot3(pathmp(:,1),pathmp(:,2),pathmp(:,3), 'DisplayName', 'mp')
% hold off
% legend

pathLLH = zeros(size(pathXYH));
for i = 1 : size(pathLLH, 1)
    pathLLH(i, :) = XY2LL(pathXYH(i, :), algoParam.start);
end

%% 绘制结果和RRT树

% figure
% scatter3(0, 0, algoParam.start(3));
% hold on
% scatter3(goalxyh(1), goalxyh(2), goalxyh(3));
% for i = 1 : iter
%     plot3(RRTree(i).path(:, 1), RRTree(i).path(:, 2), RRTree(i).path(:, 3), 'LineWidth', 1, 'Color', 'r');
% end
% hold off
% grid on
% xlabel('x')
% ylabel('y')
% zlabel('h')
% title('3dim Dubins-MP-RRT Tree')
% axis equal

% figure
% hold on
% scatter3(goalxyh(1), goalxyh(2), goalxyh(3));
% for i = 1 : iter
%     %sample = ChooseSample(algoParam, a, b, c);
%     %scatter3(sample(1), sample(2), sample(3));
%     scatter3(RRTree(i).state(1), RRTree(i).state(2), RRTree(i).state(3));
% end
% hold off
% grid on
% xlabel('x')
% ylabel('y')
% title('3dim sample')
% %axis equal

figure
geoaxes("Basemap", "satellite") % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
hold on
geoplot(pathLLH(:, 2), pathLLH(:, 1), '-', 'LineWidth', 2, 'Color', 'w', "DisplayName", "path")
for i = 1 : numel(Obslist)
    Obslist(i).plot('Color', 'r');
end
hold off
set(gca, 'TickLabelFormat', '-dd') % 十进制度，用减号 (-) 表示南纬和西经
legend
title('2dim path')

% figure
% axesm('MapProjection', 'gstereo', 'MapLatLimit', algoParam.mapRef.LatitudeLimits, 'MapLonLimit', algoParam.mapRef.LongitudeLimits, ...
%     'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on') % 创建了一个基于轴的地图，使用极射赤平投影化为矩形
% geoshow(algoParam.highData, algoParam.mapRef, 'DisplayType', 'surface')
% tightmap; % 删除基于轴的贴图周围的空白
% demcmap(algoParam.highData)
% daspectm('m', 5);
% view(20, 35)
% camlight % 创建或移动光源
% % ax = gca;
% % ax.Box = 'off'; % 不显示边框
% % axis tight % 删除基于轴的贴图周围的空白
% setm(gca, 'MLabelLocation', 0.2, 'MLabelRound', -1)
% setm(gca, 'PLabelLocation', 0.2, 'PLabelRound', -1)
% setm(gca, 'MLabelParallel', 'south')
% setm(gca, 'PLabelMeridian', 'east')
% setm(gca, 'FLineWidth', 0.5)
% plot3m(pathLLH(:, 2), pathLLH(:, 1), pathLLH(:, 3), LineWidth = 1.5, Color = 'red')
% title('3dim MP-RRT route plan')


dist2dim = vecnorm(diff(pathXYH(:, 1 : 2)), 2, 2);
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist] / 1000;
hterrain = zeros(size(cumdist));
for i = 1 : numel(hterrain)
    [I, J] = geographicToDiscrete(algoParam.mapRef, pathLLH(i, 2), pathLLH(i, 1)); % 将地理坐标转换为数组索引
    hterrain(i) = algoParam.highData(I, J);
end

dist2dim = vecnorm(diff(pathXYHnew(:, 1 : 2)), 2, 2);
cumdist1 = cumsum(dist2dim);
cumdist1 = [0; cumdist1] / 1000;

nodehp = zeros(numel(nodeList), 2); % 每个节点的高度剖面
nodehp(1, 2) = nodeList(1).state(3);
for i = 2 : numel(nodeList)
    nodehp(i, 2) = nodeList(i).state(3);
    nodehp(i, 1) = nodehp(i - 1, 1) + sum(vecnorm(diff(nodeList(i).path(:, 1 : 2)), 2, 2 )) / 1000;
end

figure
hold on
plot(cumdist, pathXYH(:, 3), 'LineWidth', 1.5, 'DisplayName', '优化前飞行高度')
plot(cumdist, hterrain, 'LineWidth', 1.5, 'DisplayName', '地面高度')
plot(cumdist1, pathXYHnew(:, 3), 'LineWidth', 1.5, 'DisplayName', '优化后飞行高度')
scatter(nodehp(:, 1), nodehp(:, 2), 'DisplayName', '节点高度')
hold off
xlabel('横向距离/km')
ylabel('飞行高度/m')
%title('飞行高度剖面')
legend('FontSize', 10)

%nodeList = GetParentNodeList(RRTree, goalind)

% for i=1:iter
%     if isequal(RRTree(i).state(1:2), [-17500 25000])
%         break;
%     end
% end
% [~, length, ~, ~] = Get3dimDubinsPath(RRTree(i).state, goalxyh, algoParam.speed, algoParam.dynamicCons, true);
% RRTree(i).cost + length



