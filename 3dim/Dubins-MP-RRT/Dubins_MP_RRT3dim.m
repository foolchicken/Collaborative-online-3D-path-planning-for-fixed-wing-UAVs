% 改进MPRRT，借鉴Accelerating Kinodynamic RRT* Through Dimensionality Reduction论文思路进行降维
% 运动基元为dubins路径
% 扩展新节点时从无终端约束数据库中选择运动基元，重布线时使用有终端约束数据库，连接到终点时直接使用dubins进行计算
% 虽然是三维空间，但是仍旧从二维数据库中寻找运动基元，高度规划使用双线性积分模型求解无约束最优控制
addpath('D:\Matlabproject\航迹规划算法')
addpath('D:\Matlabproject\硕士毕业论文\地图处理')
addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')

close all
clear
clc

load("mpdataFTAv90_2.mat")
load("mpdatav90_2.mat")
if mpdataFTA.gridres ~= mpdata.gridres || mpdataFTA.speed ~= mpdata.speed
    error('数据库分辨率或速度不同！')
end
disp('运动基元数据库加载成功！')

generateObs; % 生成障碍物

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.maxh = 3000; % 最大飞行高度

algoParam.maxIter = 200; % 最大扩展次数，即树的最大容量
algoParam.start = [109.2, 34.61, 600, deg2rad(90), deg2rad(0)];
algoParam.goal = [108.9, 34.911, 1000, deg2rad(200), deg2rad(0)];
algoParam.dynamicCons = dynamicCons;
algoParam.checkStep = 1;
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.resolutionMP = mpdata.gridres; % 运动基元网格分辨率
algoParam.speed = mpdata.speed; % 飞行速度
algoParam.dsafe = 200; % 最小离地高度
[algoParam.highData, algoParam.mapRef] = readgeoraster('D:\MapService\西安高程地图small.tif'); % 读取高程地图
algoParam.highData = double(algoParam.highData); % 统一使用double计算
algoParam.obs = Obslist;
for i = 1 : numel(Obslist)
    algoParam.obs(i) = algoParam.obs(i).LL2XY(algoParam.start(1 : 2)); % 障碍物坐标转换
end

disp('高程地图加载成功！')
disp('地图范围：')
disp(['经度 ' num2str(algoParam.mapRef.LongitudeLimits(1)) '--' num2str(algoParam.mapRef.LongitudeLimits(2))]);
disp(['纬度 ' num2str(algoParam.mapRef.LatitudeLimits(1)) '--' num2str(algoParam.mapRef.LatitudeLimits(2))]);

minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点

%% 判断起终点是否可行
if ~PointCheck(algoParam.start, algoParam)
    error('起点不在地图范围内或者位于障碍物内！');
end
if ~PointCheck(algoParam.goal, algoParam)
    error('终点不在地图范围内或者位于障碍物内！');
end

generateNodeStruct; % 生成节点数据结构
RRTree = repmat(StructNode, algoParam.maxIter, 1);

initNode = StructNode;
initNode.yawf = algoParam.start(4);
initNode.pitchf = algoParam.start(5);
initNode.pos = algoParam.start(1 : 3); % 位置为LLH坐标
initNode.path = [0 0 algoParam.start(3)]; % 路径为xyh坐标
initNode.cost = 0;
initNode.ind = 1;
initNode.parent = -1;

RRTree(1) = initNode;
n = size(mpdata.database, 1);
range = (n - 1) * mpdata.gridres; % 运动基元数据库范围
step = range / 2; % 搜索步长

disp(['运动基元数据库范围 ' num2str(range) ' m'])

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
        eucdist(i) = norm(RRTree(i).path(end, 1 : 3) - sample);
    end
    [mindist, index] = min(eucdist);
    if mindist < 2 % 距离过近，重新采样
        failedAttempts = failedAttempts + 1;
        continue;
    end

    nearestNode = RRTree(index);
    if mindist > range % 如果采样点超出了基元数据库范围
        dirxy = atan2(sample(2) - nearestNode.path(end, 2), sample(1) - nearestNode.path(end, 1));
        dirz = atan2(sample(3) - nearestNode.path(end, 3), norm(sample(1 : 2) - nearestNode.path(end, 1 : 2)));
        stepxy = step * cos(dirz);
        stepz = step * sin(dirz);
        sample(1 : 2) = nearestNode.path(end, 1 : 2) + stepxy * [cos(dirxy)  sin(dirxy)]; % 从最近点出发朝采样点前进step长度
        sample(3) = nearestNode.path(end, 3) + stepz;
        sample = NormalizedGrid(sample, algoParam); % 栅格化
    end

    %% 扩展RRT树
    nearNodes = NearNodes(sample, RRTree(1 : iter), range, algoParam); % 采样点的邻结点
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
    end
    failedAttempts = 0;
    iter = iter + 1;
    newNode.ind = iter;
    RRTree(iter) = newNode;

    %% 重新布线
    queue = zeros(int32(algoParam.maxIter / 5), 1);
    if ~isequal(sample, goalxyh(1 : 3)) % 只有采样点不是终点时才重布线
        for i = 1 : size(nearNodes, 1)
            if nearNodes(i).ind == 1 || nearNodes(i).ind == newNode.parent % 邻居节点是起点和父节点时跳过
                continue;
            end
            if isequal(nearNodes(i).path(end, 1 : 2), goalxyh(1 : 2)) % 邻节点为终点时直接计算dubins
                startPose = [newNode.path(end, :) newNode.yawf newNode.pitchf];
                [pathXYH, length, dubinsParam] = Get3dimDubinsPath(startPose, goalxyh, algoParam.speed, algoParam.dynamicCons);
                newcost = newNode.cost + length;
            else
                startPose = [newNode.path(end, :) newNode.yawf newNode.pitchf];
                goalPose = [nearNodes(i).path(end, 1 : 3) nearNodes(i).yawf nearNodes(i).pitchf];
                mp = GetMotionPrimitiveWithHigh(mpdata, startPose, goalPose, algoParam.dynamicCons, true); % 以新节点为起点，到达邻节点的运动基元，此时考虑终端角约束
                newcost = newNode.cost + mp.cost;
                if ~isinf(mp.cost)
                    pathXYH = newNode.path(end, 1 : 3) + mp.path(:, 1 : 3);
                end
            end
            if ~isinf(newcost) && PathCollisionCheck(pathXYH, algoParam)
                if newcost < nearNodes(i).cost
                    RRTree(nearNodes(i).ind).parent = newNode.ind; % 更新父节点索引
                    RRTree(nearNodes(i).ind).path = pathXYH;
                    RRTree(nearNodes(i).ind).yaw0 = newNode.yawf; % 起始航向角为新节点的终止航向角
                    RRTree(nearNodes(i).ind).pitch0 = newNode.pitchf;
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
    if isequal(goalxyh(1 : 3), sample) % 第一次找到终点
        bestLength = newNode.cost;
        goalind = iter; % 找到终点时的迭代次数
        disp(['在第 ' num2str(goalind) ' 次采样找到可行路径，长度为 ' num2str(bestLength)])
    end
    if ~isinf(bestLength) && RRTree(goalind).cost < bestLength % 找到更好的路径
        bestLength = RRTree(goalind).cost;
        disp(['找到更佳路径， 新长度为 ' num2str(bestLength)])
    end
end
toc

disp(['重布线 ' num2str(count) ' 次'])

pathXYH = FindWayBack(RRTree(1 : iter), algoParam);
pathLLH = zeros(size(pathXYH));
for i = 1 : size(pathLLH, 1)
    pathLLH(i, :) = XY2LL(pathXYH(i, :), algoParam.start);
end

%% 绘制结果和RRT树

figure
scatter3(0, 0, algoParam.start(3));
hold on
scatter3(goalxyh(1), goalxyh(2), goalxyh(3));
for i = 1 : iter
    plot3(RRTree(i).path(:, 1), RRTree(i).path(:, 2), RRTree(i).path(:, 3), 'LineWidth', 1, 'Color', 'r');
end
hold off
grid on
xlabel('x')
ylabel('y')
zlabel('h')
title('3dim Dubins-MP-RRT Tree')
axis equal

% figure
% scatter3(0, 0, algoParam.start(3));
% hold on
% scatter3(goalxyh(1), goalxyh(2), goalxyh(3));
% plot3(pathXYH(:, 1), pathXYH(:, 2), pathXYH(:, 3), 'LineWidth', 1.5, 'Color', 'r');
% hold off
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% title('3dim MP-RRT route plan')

% llh空间
% figure
% axesm('MapProjection', 'gstereo', 'MapLatLimit', algoParam.mapRef.LatitudeLimits, 'MapLonLimit', algoParam.mapRef.LongitudeLimits, ...
%      'Frame', 'on', 'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on')
% geoshow(algoParam.highData, algoParam.mapRef, 'DisplayType', 'surface')
% demcmap(algoParam.highData)
% axis off
% setm(gca, 'MLabelLocation', 0.2, 'MLabelRound', -1)
% setm(gca, 'PLabelLocation', 0.2, 'PLabelRound', -1)
% plot3m(pathLLH(:,2), pathLLH(:,1), pathLLH(:,3), LineWidth=1.5,Color='red')

figure
geoaxes("Basemap", "satellite", "ZoomLevel", 9) % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
hold on
geoplot(pathLLH(:, 2), pathLLH(:, 1), '-', 'LineWidth', 2, 'Color', 'w', "DisplayName", "path")
for i = 1 : numel(Obslist)
    Obslist(i).plot('Color', 'r');
end
hold off
legend
title('2dim path')

figure
axesm('MapProjection', 'gstereo', 'MapLatLimit', algoParam.mapRef.LatitudeLimits, 'MapLonLimit', algoParam.mapRef.LongitudeLimits, ...
    'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on') % 创建了一个基于轴的地图，使用极射赤平投影化为矩形
geoshow(algoParam.highData, algoParam.mapRef, 'DisplayType', 'surface')
tightmap; % 删除基于轴的贴图周围的空白
demcmap(algoParam.highData)
daspectm('m', 5);
view(20, 35)
camlight % 创建或移动光源
% ax = gca;
% ax.Box = 'off'; % 不显示边框
% axis tight % 删除基于轴的贴图周围的空白
setm(gca, 'MLabelLocation', 0.2, 'MLabelRound', -1)
setm(gca, 'PLabelLocation', 0.2, 'PLabelRound', -1)
setm(gca, 'MLabelParallel', 'south')
setm(gca, 'PLabelMeridian', 'east')
setm(gca, 'FLineWidth', 0.5)
plot3m(pathLLH(:, 2), pathLLH(:, 1), pathLLH(:, 3), LineWidth = 1.5, Color = 'red')
title('3dim MP-RRT route plan')


dist2dim = vecnorm(diff(pathXYH(:, 1 : 2)), 2, 2);
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist] / 1000;
hterrain = zeros(size(cumdist));
for i = 1 : numel(hterrain)
    [I, J] = geographicToDiscrete(algoParam.mapRef, pathLLH(i, 2), pathLLH(i, 1)); % 将地理坐标转换为数组索引
    hterrain(i) = algoParam.highData(I, J);
end
figure
hold on
plot(cumdist, pathXYH(:, 3), 'LineWidth', 1.5, 'DisplayName', '飞行高度')
plot(cumdist, hterrain, 'LineWidth', 1.5, 'DisplayName', '地面高度')
hold off
xlabel('横向距离 km')
ylabel('飞行高度 m')
title('飞行高度剖面')
legend

% n = 300000;
% tc = zeros(n, 1);
% for i = 1 : n
%     tic
%     Obslist(1).ifinObs([0, 0]);
%     tc(i) = toc;
% end
% sum(tc) / n * 1000


