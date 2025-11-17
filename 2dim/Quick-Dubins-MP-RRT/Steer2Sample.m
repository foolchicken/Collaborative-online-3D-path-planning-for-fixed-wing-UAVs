function [newNode, errornum] = Steer2Sample(qs, nearNodes, dataBase, algoParam)
% 向新的采样点连线，从邻节点中选择最好的节点作为父节点
% 即把连线和重新选择父节点两个步骤合二为一

newNode = [];
cbest = inf; % 起点到当前接节点的最佳代价

% mapSize = size(algoParam.map); % 栅格地图大小
% ind = round(qs / algoParam.resolutionMap) + 1;
% if algoParam.map(ind(2), ind(1)) == 0 % 采样点不可行返回
%     errornum = 1;
%     return
% end
% ind = round(qs - [0, algoParam.dsafe] / algoParam.resolutionMap) + 1;
% if ind(2) < 1, ind(2) = 1; end
% if algoParam.map(ind(2), ind(1)) == 0 % 采样点不可行返回
%     return
% end
% ind = round(qs + [0, algoParam.dsafe] / algoParam.resolutionMap) + 1;
% if ind(2) > mapSize(1), ind(2) = mapSize(1); end
% if algoParam.map(ind(2), ind(1)) == 0 % 采样点不可行返回
%     return
% end
% ind = round(qs - [algoParam.dsafe, 0] / algoParam.resolutionMap) + 1;
% if ind(1) < 1, ind(1) = 1; end
% if algoParam.map(ind(2), ind(1)) == 0 % 采样点不可行返回
%     return
% end
% ind = round(qs + [algoParam.dsafe, 0] / algoParam.resolutionMap) + 1;
% if ind(1) > mapSize(2), ind(1) = mapSize(2); end
% if algoParam.map(ind(2), ind(1)) == 0 % 采样点不可行返回
%     return
% end

ifexistmp = false;
for i = 1 : size(nearNodes, 1)
    if isequal(nearNodes(i).pos, algoParam.goal(1 : 2)) % 邻节点是终点时不连线
        continue
    end

    %motionPrimitive = GetMotionPrimitive(dataBase, [nearNodes(i).pos nearNodes(i).thetaf], qs, false);
    motionPrimitive = GetMotionPrimitiveTF(dataBase, [nearNodes(i).pos nearNodes(i).thetaf], qs);
    if ~isempty(motionPrimitive) && nearNodes(i).cost + motionPrimitive.cost < cbest
        ifexistmp = true; % 存在基元
        path = motionPrimitive.path(:,1:2);
        path = path + nearNodes(i).pos; % nearNodes到qs的路径
        if PathCollisionCheck(path, algoParam)
            cbest = nearNodes(i).cost + motionPrimitive.cost; % 父节点代价+新路径代价
            newNode = motionPrimitive;
            newNode.path = path;
            newNode.cost = cbest;
            newNode.parent = nearNodes(i).ind;
            newNode.pos = qs;
            newNode.h = norm(qs - algoParam.goal(1 : 2));
            newNode.ifact = true;
        end
    end
end

if isempty(newNode)
    if ifexistmp
        errornum = 2; % 是因为路径碰撞才无解
    else
        errornum = 3; % 因为没有基元才无解
    end
else
    errornum = 0;
end

end
