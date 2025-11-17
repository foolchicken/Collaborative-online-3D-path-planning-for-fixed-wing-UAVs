function newNode = Steer2Goal(nearNodes, algoParam)
% 向目标点连线

cbest = inf; % 起点到当前接节点的最佳代价
newNode = [];
for i = 1 : size(nearNodes, 1)
    startPose = [nearNodes(i).path(end, 1 : 3) nearNodes(i).yawf nearNodes(i).pitchf];
    goalxyh = LL2XY(algoParam.goal, algoParam.start);
    [path, length, dubinsParam] = Get3dimDubinsPath(startPose, goalxyh, algoParam.speed, algoParam.dynamicCons);
    if nearNodes(i).cost + length < cbest
        if PathCollisionCheck(path, algoParam)
            cbest = nearNodes(i).cost + length;
            newNode.yaw0 = nearNodes(i).yawf;
            newNode.yawf = algoParam.goal(4);
            newNode.pitch0 = nearNodes(i).pitchf;
            newNode.pitchf = algoParam.goal(5);
            newNode.pos = algoParam.goal(1:3);
            newNode.path = path;
            newNode.cost = cbest;
            newNode.parent = nearNodes(i).ind;
        end
    end
end


