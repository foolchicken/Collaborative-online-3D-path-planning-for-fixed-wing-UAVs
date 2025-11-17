close all
clear
clc

addpath('D:\Matlabproject\航迹规划算法')
addpath('D:\Matlabproject\dubins编队集结')
addpath('D:\Matlabproject\航迹规划算法\Dubins-RRT')

I = imread('map2.png');
%Ig = rgb2gray(I); % 将 RGB 图像或颜色图转换为灰度图

%algoParam.map = imbinarize(Ig); % 栅格为1表示可行，为0表示不可行
algoParam.map = I;
algoParam.maxIter = 1000; % 最大扩展次数，即树的最大容量
algoParam.start = [100 * 10, 40 * 10, deg2rad(90)];
algoParam.goal = [476 * 10, 420 * 10, deg2rad(0)];
algoParam.maxstep = 500; % 最大步长
algoParam.disTh = 1000; % 邻居判断距离
algoParam.checkStep = 1;
algoParam.dsafe = 2; % 到障碍物的最短距离
algoParam.maxFailedAttempts = 200; % 节点最大扩展失败次数
algoParam.resolutionMap = 10; % 栅格地图网格分辨率
algoParam.r = 160; % dubins转弯半径

%% 判断起终点是否可行
if ~PointCheck(algoParam.start, algoParam.map, algoParam.resolutionMap)
    error('起点不在地图范围内或者位于障碍物内！');
end
if ~PointCheck(algoParam.goal, algoParam.map, algoParam.resolutionMap)
    error('终点不在地图范围内或者位于障碍物内！');
end

generateNodeStruct;

nbench = 100; % 测试次数

benchRecordDRRT = [];
parfor b = 1 : nbench
    runtime = zeros(algoParam.maxIter, 1);
    cost = inf * ones(algoParam.maxIter, 1);
    RRTree = repmat(StructNode, algoParam.maxIter, 1);
    initNode = StructNode;

    initNode.cost = 0;
    initNode.ind = 1;
    initNode.parent = -1;
    initNode.pose = algoParam.start;
    initNode.path = algoParam.start;
    RRTree(1) = initNode;

    %% 进入迭代
    iter = 1;
    failedAttempts = 0;
    count = 0;
    bestLength = inf;
    tic
    while iter < algoParam.maxIter% && failedAttempts < algoParam.maxFailedAttempts
        %% 采样
        sample = ChooseSample(algoParam); % 选择采样点

        dubinsdist = zeros(iter, 1); % 采样点到RRT树的dubins距离
        for i = 1 : iter
            dubinsdist(i) = norm(RRTree(i).pose - sample); % 欧式距离
        end
        [mindist, index] = min(dubinsdist);
        nearestNode = RRTree(index);
        if mindist > algoParam.maxstep
            dir = atan2(sample(2) - nearestNode.pose(2), sample(1) - nearestNode.pose(1));
            sample(1 : 2) = nearestNode.pose(1 : 2) + algoParam.maxstep * [cos(dir)  sin(dir)]; % 从最近点出发朝采样点前进step长度
            for i = 1 : iter  % 重新计算采样点到RRT树的dubins距离
                dubinsdist(i) = norm(RRTree(i).pose - sample);
            end
            [mindist, index] = min(dubinsdist);
        end

        if mindist < 2 % 距离过近，重新采样
            failedAttempts = failedAttempts + 1;
            continue;
        end
        %% 扩展RRT树
        newNode = Steer(sample, nearestNode, algoParam);

        if ~PathCollisionCheck(newNode.path, algoParam)
            failedAttempts = failedAttempts + 1;
            continue;
        end

        failedAttempts = 0;
        iter = iter + 1;
        newNode.ind = iter;

        neighboorIndex = find(dubinsdist <= algoParam.disTh); % newPoint的邻居节点索引

        %% 重新选择父节点
        for i = 1 : numel(neighboorIndex)
            param = dubins_core(RRTree(neighboorIndex(i)).pose, newNode.pose, algoParam.r);
            length = dubins_length(param);
            nseg = max(length / algoParam.r * 4, 5); % 路径分割的线段数量
            path = dubins_path_sample_many(param, length / nseg);
            if PathCollisionCheck(path, algoParam)
                newcost = RRTree(neighboorIndex(i)).cost + length; % 以neighboor为父节点，从起点到newPoint的代价值c
                if newcost < newNode.cost % 更新父节点和适应度
                    nearestNode = RRTree(neighboorIndex(i), :);
                    newNode.cost = newcost;
                    newNode.parent = neighboorIndex(i);
                    newNode.path = path;
                end
            end
        end

        %% 重新布线
        queue = zeros(int32(algoParam.maxIter / 5), 1);
        for i = 1 : numel(neighboorIndex)
            if neighboorIndex(i) == 1 || neighboorIndex(i) == newNode.parent % 邻居节点是起点和父节点时跳过
                continue
            end
            param = dubins_core(newNode.pose, RRTree(neighboorIndex(i)).pose, algoParam.r);
            length = dubins_length(param);
            newcost = newNode.cost + length; % 重新布线的适应度，以newPoint为父节点，从起点到neighboor的代价值c
            nseg = max(length / algoParam.r * 4, 5); % 路径分割的线段数量
            path = dubins_path_sample_many(param, length / nseg);

            if newcost < RRTree(neighboorIndex(i)).cost && PathCollisionCheck(path, algoParam)

                RRTree(neighboorIndex(i)).parent = newNode.ind; % 更新父节点索引
                RRTree(neighboorIndex(i)).path = path; % 更新路径
                % 要后驱改变neighboor的所有子节点适应度
                deltaCost = newcost - RRTree(neighboorIndex(i)).cost;
                bottom = 0;
                top = 0;
                bottom = bottom + 1;
                queue(bottom) = neighboorIndex(i);
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
        RRTree(iter, :) = newNode; % 拓展节点

        if isequal(algoParam.goal, sample) % 第一次找到终点
            bestLength = newNode.cost;
            goalind = iter; % 找到终点时的迭代次数
            %disp(['在第 ' num2str(goalind) ' 次采样找到可行路径，长度为 ' num2str(bestLength)])
            %toc
        end
        if ~isinf(bestLength) && RRTree(goalind).cost < bestLength % 找到更好的路径
            bestLength = RRTree(goalind).cost;
            %disp(['找到更佳路径， 新长度为 ' num2str(bestLength)])
        end
        runtime(iter - 1) = toc;
        cost(iter - 1) = bestLength;
    end
    disp(['最终长度为 ' num2str(bestLength)])
    benchRecordDRRT(b).runtime = runtime;
    benchRecordDRRT(b).cost = cost;
end
save('benchRecordDRRT.mat', 'benchRecordDRRT', '-v7.3')

% disp(['重布线 ' num2str(count) ' 次'])
% 
% solution = FindWayBack(RRTree, algoParam);
% 
% figure
% plotGrid(algoParam.map)
% %xticks = get(gca, 'XTick');
% xticklabels({'0', '1', '2', '3', '4', '5'});
% %yticks = get(gca, 'YTick');
% yticks([0 100 200 300 400 500])
% yticklabels({'0', '1', '2', '3', '4', '5'});
% hold on
% scatter(algoParam.start(1) / algoParam.resolutionMap, ...
%     algoParam.start(2) / algoParam.resolutionMap);
% scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
%     algoParam.goal(2) / algoParam.resolutionMap);
% for i = 1 : iter
%     plot(RRTree(i).path(:, 1) / algoParam.resolutionMap, ...
%         RRTree(i).path(:, 2) / algoParam.resolutionMap, 'LineWidth', 1, 'Color', 'r');
% end
% hold off
% grid on
% xlabel('x/km')
% ylabel('y/km')
% title('Dubins-RRT Tree')
% 
% figure
% plotGrid(algoParam.map)
% hold on
% scatter(algoParam.start(1) / algoParam.resolutionMap, ...
%     algoParam.start(2) / algoParam.resolutionMap);
% scatter(algoParam.goal(1) / algoParam.resolutionMap, ...
%     algoParam.goal(2) / algoParam.resolutionMap );
% plot(solution(:, 1) / algoParam.resolutionMap, ...
%     solution(:, 2) / algoParam.resolutionMap, 'LineWidth', 1.5, 'Color', 'r');
% % plot(path(:, 1) , path(:, 2))
% hold off
% grid on
% axis equal
% xlabel('x')
% ylabel('y')
% title('Dubins-RRT route plan')

