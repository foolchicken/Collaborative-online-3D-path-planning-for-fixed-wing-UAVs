function newNode = Steer2Sample(qs, nearNodes, dataBase, algoParam)
% 向新的采样点连线，从邻节点中选择最好的节点作为父节点
% 即把连线和重新选择父节点两个步骤合二为一

newNode = [];
cbest = inf; % 起点到当前接节点的最佳代价
for i = 1 : size(nearNodes, 1)
    if isequal(nearNodes(i).path(end, 1 : 2), algoParam.goal(1 : 2)) % 邻节点是终点时不连线
        continue
    end
    motionPrimitive = GetMotionPrimitive(dataBase, [nearNodes(i).path(end, :) nearNodes(i).thetaf], qs, false);
    if ~isempty(motionPrimitive) && nearNodes(i).cost + motionPrimitive.cost < cbest
        path = motionPrimitive.path(:, 1 : 2) + nearNodes(i).path(end, 1 : 2); % nearNodes到qs的路径
        if PathCollisionCheck(path, algoParam)
            cbest = nearNodes(i).cost + motionPrimitive.cost; % 父节点代价+新路径代价
            newNode = motionPrimitive;
            newNode.path = path;
            newNode.cost = cbest;
            newNode.parent = nearNodes(i).ind;
        end
    end
end

end
