close all
clear
clc

addpath('D:\Matlabproject\航迹规划算法\Dubins-RRT')
addpath('D:\Matlabproject\硕士毕业论文\3dim')
addpath('D:\Matlabproject\硕士毕业论文\地图处理')

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.maxh = 5000; % 最大飞行高度

algoParam.maxIter = 200; % 最大扩展次数，即树的最大容量
algoParam.maxstep = 5000; % 最大步长
algoParam.disTh = 13000; % 邻居判断距离
algoParam.start = [101.3, 38.8, 3000, deg2rad(90), deg2rad(0)]; % 经纬高 航向角 俯仰角
algoParam.goal = [101.5, 39.2, 3000, deg2rad(90), deg2rad(0)];
algoParam.checkStep = 3;
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.dynamicCons = dynamicCons;
algoParam.speed = 100; % 飞行速度
[algoParam.highData, algoParam.mapRef] = readgeoraster('D:\MapService\small1.tif'); % 读取高程地图

originPoint = algoParam.start(1 : 3); % 以起点作为发射系原点

%% 判断起终点是否可行
if ~PointCheck(algoParam.start, algoParam)
    error('起点不在地图范围内或者位于障碍物内！');
end
if ~PointCheck(algoParam.goal, algoParam)
    error('终点不在地图范围内或者位于障碍物内！');
end

StructNode.cost = []; % 从起始节点到当前节点的路径代价
StructNode.ind = []; % 节点自身在树中的索引
StructNode.parent = []; % 父节点在树中的索引
StructNode.path = []; % 父节点到当前节点的路径（计算过程中使用xyz坐标）
StructNode.pose = []; % 当前节点的位姿

RRTree = repmat(StructNode, algoParam.maxIter, 1);
initNode = StructNode;

initNode.cost = 0;
initNode.ind = 1;
initNode.parent = -1;
initNode.pose = algoParam.start;
RRTree(1) = initNode;

RRTree = repmat(StructNode, algoParam.maxIter, 1);
initNode = StructNode;

initNode.cost = 0;
initNode.ind = 1;
initNode.parent = -1;
initNode.pose = [0 0 0 algoParam.start(4 : end)]; % 以起点为发射系原点，所以起点的xyz为0 0 0
RRTree(1) = initNode;

%% 进入迭代
iter = 1;
failedAttempts = 0;

tic
while iter < algoParam.maxIter && failedAttempts < algoParam.maxFailedAttempts
    sample = ChooseSample(algoParam); % 选择采样点

    [nearestNode, disance] = NearestNode(sample, RRTree(1 : iter), algoParam); % 由于是dubins距离，所以需要重新计算最近节点
    if isempty(nearestNode) || disance(nearestNode.ind) < 1 % 无解或者距离过小认为是同一个点认为采样失败
        failedAttempts = failedAttempts + 1;
        continue;
    end

    if norm(nearestNode.pose(1 : 3) - sample(1 : 3)) > algoParam.maxstep
        dirxy = atan2(sample(2) - nearestNode.pose(2), sample(1) - nearestNode.pose(1));
        dirz = atan2(sample(3) - nearestNode.pose(3), norm(sample(1 : 2) - nearestNode.pose(1 : 2)));
        stepxy = algoParam.maxstep * cos(dirz);
        stepz = algoParam.maxstep * sin(dirz);
        sample(1 : 2) = nearestNode.pose(1 : 2) + stepxy * [cos(dirxy)  sin(dirxy)]; % 从最近点出发朝采样点前进step长度
        sample(3) = nearestNode.pose(3) + stepz;
        [nearestNode, disance] = NearestNode(sample, RRTree(1 : iter), algoParam); % 由于是dubins距离，所以需要重新计算最近节点
    end
    [path, length, dubinsParam] = Get3dimDubinsPath(nearestNode.pose, sample, algoParam.speed, algoParam.dynamicCons);

    if isinf(length)
        failedAttempts = failedAttempts + 1;
        continue;
    end
    pathLLH = XYZ2LLHPath(path(1 : algoParam.checkStep : end, :), originPoint);
    if ~PathCollisionCheck(pathLLH, algoParam)
        failedAttempts = failedAttempts + 1;
        continue;
    end

    failedAttempts = 0;
    iter = iter + 1;

    newNode = StructNode;
    newNode.ind = iter;
    newNode.parent = nearestNode.ind;
    newNode.pose = sample;
    newNode.path = path(1:algoParam.checkStep:end,:); %DouglasPeucker(path, 10); % 抽稀后存储
    newNode.cost = nearestNode.cost + length;

    neighboorIndex = find(disance <= algoParam.disTh); % newPoint的邻居节点索引

    %% 重新选择父节点
    for i = 1 : numel(neighboorIndex)
        [path, length, dubinsParam] = Get3dimDubinsPath(RRTree(neighboorIndex(i)).pose, newNode.pose, algoParam.speed, algoParam.dynamicCons);
        if isinf(length), continue; end
        pathLLH = XYZ2LLHPath(path(1 : algoParam.checkStep : end, :), originPoint);
        if PathCollisionCheck(pathLLH, algoParam)
            newcost = RRTree(neighboorIndex(i)).cost + length; % 以neighboor为父节点，从起点到newPoint的代价值c
            if newcost < newNode.cost % 更新父节点和适应度
                nearestNode = RRTree(neighboorIndex(i), :);
                newNode.cost = newcost;
                newNode.parent = neighboorIndex(i);
                newNode.path = DouglasPeucker(path, 10);
            end
        end
    end

    %% 重新布线
    queue = zeros(int32(algoParam.maxIter / 5), 1);
    for i = 1 : numel(neighboorIndex)
        if neighboorIndex(i) == 1 || neighboorIndex(i) == newNode.parent % 邻居节点是起点和父节点时跳过
            continue
        end
        [path, length, dubinsParam] = Get3dimDubinsPath(newNode.pose, RRTree(neighboorIndex(i)).pose, algoParam.speed, algoParam.dynamicCons);
        if isinf(length), continue; end
        pathLLH = XYZ2LLHPath(path(1 : algoParam.checkStep : end, :), originPoint);
        newcost = newNode.cost + length; % 重新布线的适应度，以newPoint为父节点，从起点到neighboor的代价值c
        if newcost < RRTree(neighboorIndex(i)).cost && PathCollisionCheck(pathLLH, algoParam)

            RRTree(neighboorIndex(i)).parent = newNode.ind; % 更新父节点索引
            RRTree(neighboorIndex(i)).path = DouglasPeucker(path, 10); % 更新路径
            % 要后驱改变neighboor的所有子节点适应度
            deltaCost = newcost - RRTree(neighboorIndex(i)).cost;
            bottom = 0;
            top = 0;
            bottom = bottom + 1;
            queue(bottom) = neighboorIndex(i);

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


    RRTree(iter, :) = newNode; % 拓展节点

end
toc

solution = FindWayBack(RRTree, algoParam);

[goal(1), goal(2), goal(3)] = geodetic2enu(algoParam.goal(2), algoParam.goal(1), algoParam.goal(3), originPoint(2), originPoint(1), originPoint(3), wgs84Ellipsoid);
figure
scatter(0, 0, 'black')
hold on
scatter(goal(1), goal(2), 'red')
for i = 2 : size(RRTree, 1)
    %pathLLH = XYZ2LLHPath(RRTree(i).path, originPoint);
    pathLLH = RRTree(i).path;
    plot(pathLLH(:, 1), pathLLH(:, 2), 'LineWidth', 1, 'Color', 'r');
    scatter3(RRTree(i).pose(1), RRTree(i).pose(2), RRTree(i).pose(3), 'blue');
end
grid on
axis equal
hold off

figure
scatter(0, 0, 'black')
hold on
scatter(goal(1), goal(2), 'red')
plot(solution(:,1), solution(:,2), 'LineWidth', 1.5,'Color','r')
grid on
axis equal
hold off

% figure
% scatter3(0, 0, 0, 'black')
% hold on
% scatter3(goal(1), goal(2), goal(3), 'red')
% for i = 2 : size(RRTree, 1)
%     %pathLLH = XYZ2LLHPath(RRTree(i).path, originPoint);
%     pathLLH = RRTree(i).path;
%     plot3(pathLLH(:, 1), pathLLH(:, 2), pathLLH(:, 3), 'LineWidth', 1, 'Color', 'r');
%     %scatter3(RRTree(i).pose(1), RRTree(i).pose(2), RRTree(i).pose(3), 'blue');
% end
% grid on
% axis equal

% figure;
% ShowTIFmap('D:\MapService\test2.tif')
% hold on
% plot3m(algoParam.start(1), algoParam.start(2), algoParam.start(3));
% plot3m(algoParam.goal(1), algoParam.goal(2), algoParam.goal(3));
% for i = 2 : size(RRTree, 1)
%     pathLLH = XYZ2LLHPath(RRTree(i).path, originPoint);
%     plot3m(pathLLH(:, 1), pathLLH(:, 2), pathLLH(:,3), 'LineWidth', 1.5, 'Color', 'r');
% end
% hold off
% grid on
% xlabel('x')
% ylabel('y')
% title('3dim Dubins-RRT Tree')
%
% axesm sinusoid; framem; view(3)
% [lats,longs] = interpm([45 -45 -45 45 45 -45]',...
%                        [-100 -100 100 100 -100 -100]',1);
% z = (1:671)'/100;
% plot3m(lats(1),longs(1),z(1),'m', 'Color','r')
% plot3m(pathLLH(:,1),pathLLH(:,2),z(1:size(pathLLH,1)),'m')

[dist1,~] = distance(39, 100,39,101,wgs84Ellipsoid)
[dist2,~] = distance(40, 100,40,101,wgs84Ellipsoid)

(dist1-dist2)/dist1*100
