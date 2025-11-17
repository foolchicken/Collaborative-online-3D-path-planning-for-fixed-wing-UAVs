function newNode = Steer2Goal(nearNodes, algoParam)
% 向目标点连线

cbest = inf; % 起点到当前接节点的最佳代价
newNode = [];
for i = 1 : size(nearNodes, 1)
    goalxyh = LL2XY(algoParam.goal, algoParam.start);
    [path, length, dubinsParam] = Get3dimDubinsPath(nearNodes(i).state, goalxyh, algoParam.speed, algoParam.dynamicCons);
    if nearNodes(i).cost + length < cbest
        algoParamt = algoParam;
        algoParamt.obs = algoParamt.obs(nearNodes(i).obs);
        if PathCollisionCheck(path, algoParamt)
            cbest = nearNodes(i).cost + length;
            dh = nearNodes(i).dh;
            newNode.path = path;
            newNode.cost = cbest;
            newNode.parent = nearNodes(i).ind;
        end
    end
end

if ~isempty(newNode)
    newNode.state = goalxyh;
    newNode.pos = algoParam.goal(1 : 3);
    deltah = diff(newNode.path(:, 3));
    newNode.dh = dh + sum(abs(deltah)) + sum(abs(diff(deltah))); % 一阶差分和二阶差分绝对值之和
    newNode.h = 0;
    newNode.ifact = true;
end

end
