function newNode = Steer2Goal(nearNodes, algoParam)
% 向目标点连线

minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));
cbest = inf; % 起点到当前接节点的最佳代价
newNode = [];
for i = 1 : size(nearNodes, 1)
    startPose = [nearNodes(i).path(end, 1 : 2) nearNodes(i).thetaf];
    param = dubins_core(startPose, algoParam.goal, minR);
    length = dubins_length(param);

    if nearNodes(i).cost + length < cbest
        nseg = max(length / minR * 4, 5); % 路径分割的线段数量
        path = dubins_path_sample_many(param, length / nseg);
        path = path(:,1:2);
        path = [path; algoParam.goal(1:2)];
        if PathCollisionCheck(path, algoParam)
            cbest = nearNodes(i).cost + length;
            newNode.theta0 = nearNodes(i).thetaf;
            newNode.thetaf = algoParam.goal(3);
            newNode.path = path;
            newNode.cost = cbest;
            newNode.parent = nearNodes(i).ind;
            newNode.h = 0;
            newNode.pos = algoParam.goal(1:2);
            newNode.ifact = true;
        end
    end
end


