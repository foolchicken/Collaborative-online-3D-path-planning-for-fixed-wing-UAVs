function [allPathXYH, allPathLLH] = AdjUAVpathLength(uavPath, algoParam, mapdir)
% 调整各无人机路径长度使其相等

algoParam.start = [];
algoParam.goal = [];
[algoParam.highData, algoParam.mapRef] = readgeoraster(mapdir); % 读取高程地图
algoParam.highData = double(algoParam.highData); % 统一使用double计算

nuav = numel(uavPath);
pathlen = zeros(nuav, 1);
for i = 1 : nuav
    pathlen(i) = uavPath(i).nodeList{end}(end).cost;
end
maxLen = max(pathlen);

allPathXYH = cell(nuav, 1);
allPathLLH = cell(nuav, 1);
for i = 1 : nuav
    [allPathXYH{i}, allPathLLH{i}] = AdjsingleUAVpathLength(uavPath(i), maxLen, algoParam);
end

end

function [pathXYH, pathLLH] = AdjsingleUAVpathLength(uavPath, lenexp, algoParam)

generateObs; % 生成障碍物
algoParam.start = uavPath.nodeList{1}(1).pos; % 起点
algoParam.obs = Obslist;
for i = 1 : numel(Obslist)
    algoParam.obs(i) = algoParam.obs(i).LL2XY(algoParam.start(1 : 2)); % 障碍物坐标转换
end
% pathXYH = [];
% nodeList = uavPath.nodeList{end};
% for i = 2 : numel(nodeList)
%     pathXYH = [pathXYH; nodeList(i).path(1 : end - 1, :)];
% end
% pathXYH = [pathXYH; nodeList(end).state(1 : 3)];
% pathLLH = zeros(size(pathXYH));
% for i = 1 : size(pathXYH, 1)
%     pathLLH(i, :) = XY2LL(pathXYH(i, :), algoParam.start);
% end
% return



for ii = size(uavPath.nodeList, 1) : -1 : 1
    if lenexp < uavPath.nodeList{ii}(end).cost % 期望长度小于当前路径长度，跳过
        continue
    end
    deltaLen = lenexp - uavPath.nodeList{ii}(end).cost;
    nodeList = uavPath.nodeList{ii};
    pathXYH = [];
    for i = 2 : numel(nodeList)
        adjpathL = [];
        adjpathR = [];
        if deltaLen > 0 % 需要调整的长度不为0
            [adjpathL, infoL] = AdjDubinsPathLength3dimh(nodeList(i - 1).state, nodeList(i).state, algoParam.speed, algoParam.dynamicCons, deltaLen, 'L');
            [adjpathR, infoR] = AdjDubinsPathLength3dimh(nodeList(i - 1).state, nodeList(i).state, algoParam.speed, algoParam.dynamicCons, deltaLen, 'R');
            algoParam_ = algoParam;
            algoParam_.obs = algoParam_.obs(nodeList(i - 1).obs);
        end
        if ~isempty(adjpathL) && PathCollisionCheck(adjpathL, algoParam_) % 左侧绕飞
            pathXYH = [pathXYH; adjpathL(1 : end - 1, :)];
            if infoL >= 0 % 已经满足绕飞
                deltaLen = 0;
            else
                deltaLen = deltaLen - (sum(vecnorm(diff(adjpathL), 2, 2)) - (nodeList(i).cost - nodeList(i - 1).cost));
            end
        elseif ~isempty(adjpathR) && PathCollisionCheck(adjpathR, algoParam_) % 右侧绕飞
            pathXYH = [pathXYH; adjpathR(1 : end - 1, :)];
            if infoR >= 0 % 已经满足绕飞
                deltaLen = 0;
            else
                deltaLen = deltaLen - (sum(vecnorm(diff(adjpathR), 2, 2)) - (nodeList(i).cost - nodeList(i - 1).cost));
            end
        else % 不能绕飞，路径不变
            pathXYH = [pathXYH; nodeList(i).path(1 : end - 1, :)];
        end
    end
    pathXYH = [pathXYH; nodeList(end).state(1 : 3)];
    if deltaLen == 0
        break;
    end
end
if deltaLen > 0
    pathXYH = [];
    warning("无法完成绕飞")
    nodeList = uavPath.nodeList{end};
    for i = 2 : numel(nodeList)
        pathXYH = [pathXYH; nodeList(i).path(1 : end - 1, :)];
    end
    pathXYH = [pathXYH; nodeList(end).state(1 : 3)];
end

pathLLH = zeros(size(pathXYH));
for i = 1 : size(pathXYH, 1)
    pathLLH(i, :) = XY2LL(pathXYH(i, :), algoParam.start);
end

end







