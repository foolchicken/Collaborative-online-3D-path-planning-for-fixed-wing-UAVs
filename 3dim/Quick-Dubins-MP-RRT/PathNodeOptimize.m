function [nodeList, RRTree] = PathNodeOptimize(RRTree, ind, algoParam)
% 路径节点优化，输入RRT树、节点索引，输出优化后的节点列表和RRT树

nodeList = [];
child = RRTree(ind);
parent = child.parent;
while parent > 0
    nodeList = [child; nodeList];
    child = RRTree(parent);
    parent = child.parent;
end
nodeList = [RRTree(1); nodeList];

nodeList = PathVerticalOptimize(nodeList, algoParam); % 节点高度和俯仰角优化
minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点

%% 删除多余节点
lastReserced = 1; % 上一个被保留的节点
for i = 2 : numel(nodeList)
%     nodeList(i).pos(3) = nodeh(i);
%     nodeList(i).state(3) = nodeh(i);

    parentind = lastReserced; % 父节点为上一次被保留的节点
%     if i + 1 <= numel(nodeList)
%         pathseg1 = DubinsPath([nodeList(parentind).state(1 : 2), nodeList(parentind).state(4)], [nodeList(i).state(1 : 2), nodeList(i).state(4)], minR);
%         pathseg2 = DubinsPath([nodeList(i).state(1 : 2), nodeList(i).state(4)], [nodeList(i + 1).state(1 : 2), nodeList(i + 1).state(4)], minR);
%         if pathseg1.MotionLengths(3) / minR + pathseg2.MotionLengths(1) / minR < deg2rad(10)
%             isreserve = false;
%         else
%             isreserve = true;
%         end
%         if nodeList(i).state(3) > nodeList(i + 1).state(3)
%             isreserve = true;
%         end
%         if isreserve
%             lastReserced = i;
%         end
%     end
    lastReserced = i;

    [pathseg, length] = Get3dimDubinsPath(nodeList(parentind).state, nodeList(i).state, algoParam.speed, algoParam.dynamicCons); % 要约束终端俯仰角！
    if isempty(pathseg)
        warning("路径为空")
    end
    %     if abs(pitchf - nodeList(i).state(5)) < 1e-2 && abs(length - (nodeList(i).cost - RRTree(nodeList(i).parent).cost)) < 1
    %         continue % 优化不明显跳过（貌似不能跳过）
    %     end
    nodeList(i).path = pathseg;
    nodeList(i).parent = nodeList(parentind).ind;
    nodeList(i).h = norm(nodeList(i).state(1 : 3) - goalxyh(1 : 3));
    nodeList(i).cost = nodeList(parentind).cost + length;
    deltah = diff(pathseg(:, 3));
    nodeList(i).dh = nodeList(parentind).dh + sum(abs(deltah)) + sum(abs(diff(deltah)));

    RRTree(nodeList(i).ind) = nodeList(i);
    % 要后驱改变 nodeList(i)的所有子节点到该点的路径和相应的deltah deltacost
    for j = 1 : size(RRTree, 1)
        if i + 1 <= numel(nodeList) && RRTree(j).parent == nodeList(i).ind && RRTree(j).ind ~= nodeList(i+1).ind  % 还会影响被重布线节点的一级子节点
            [newpath, length, ~, ~] = Get3dimDubinsPath(nodeList(i).state, RRTree(j).state, algoParam.speed, algoParam.dynamicCons);

            if isempty(newpath)
                warning("路径为空")
                deltadh = inf;
                length = inf;
            else
                deltah = diff(newpath(:, 3));
                deltadh = nodeList(i).dh + sum(abs(deltah)) + sum(abs(diff(deltah))) - RRTree(j).dh;
            end
            RRTree(j).path = newpath;
            deltaCost = nodeList(i).cost + length - RRTree(j).cost;

            fun = @(node) ChangeNodeCostdh(node, deltaCost, deltadh);
            RRTree = TraversalRRTree(RRTree, j, fun);
        end
    end
    if RRTree(ind).dh < nodeList(i).state(3) - algoParam.start(3)
        ind
    end
end

nodeList = GetParentNodeList(RRTree, ind);

% path = [];
% 
% for i = 1 : numel(nodeList)
%     path = [path; nodeList(i).path(1 : end - 1, :)];
% end
% path = [path; goalxyh(1 : 3)];
% if ~PathCollisionCheck(path, algoParam)
%     warning("路径碰撞！")
% end

end




