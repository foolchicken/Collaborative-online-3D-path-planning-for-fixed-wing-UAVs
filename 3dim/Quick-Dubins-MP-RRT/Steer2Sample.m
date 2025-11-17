function newNode = Steer2Sample(qs, nearNodes, dataBase, algoParam)
% 向新的采样点连线，从邻节点中选择最好的节点作为父节点
% 即把连线和重新选择父节点两个步骤合二为一

newNode = [];
cbest = inf; % 起点到当前接节点的最佳代价
if ~PointCheck(qs, algoParam)
    return
end
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点
for i = 1 : size(nearNodes, 1)
    if isequal(nearNodes(i).state(1 : 2), LL2XY(algoParam.goal(1 : 2), algoParam.start)) % 邻节点是终点时不连线
        continue
    end
    motionPrimitive = GetMotionPrimitiveWithHigh(dataBase, nearNodes(i).state, qs, algoParam.dynamicCons, false);
    if ~isinf(motionPrimitive.cost) && nearNodes(i).cost + motionPrimitive.cost < cbest
        path = motionPrimitive.path; % nearNodes到qs的路径
        algoParamt = algoParam;
        algoParamt.obs = algoParamt.obs(nearNodes(i).obs);
        if PathCollisionCheck(path, algoParamt)
            cbest = nearNodes(i).cost + motionPrimitive.cost; % 父节点代价+新路径代价

            newNode.state = [qs(1 : 3), motionPrimitive.yawf, motionPrimitive.pitchf];
            newNode.path = path;
            newNode.cost = cbest;
            newNode.parent = nearNodes(i).ind;
            dh = nearNodes(i).dh;
        end
    end
end

if ~isempty(newNode)
    newNode.pos = XY2LL(qs(1 : 3), algoParam.start);
    newNode.h = norm(qs(1 : 3) - goalxyh(1 : 3));
    newNode.ifact = true;
    deltah = diff(newNode.path(:, 3));
    newNode.dh = dh + sum(abs(deltah)) + sum(abs(diff(deltah))); % 一阶差分和二阶差分绝对值之和
end

end
