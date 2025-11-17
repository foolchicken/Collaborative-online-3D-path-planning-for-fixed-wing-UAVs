function newNode = Steer2Sample(sample, nearestNode, RRTree, algoParam)
% 从nearestNode出发向sample连线

ntemp = RRTree(nearestNode.parent);
[path, length, maxk] = GetBspline2Order(ntemp.pos, nearestNode.pos, sample);

newNode.parent = nearestNode.ind;
newNode.pos = sample;
newNode.path = path;

if isinf(maxk) || maxk > 1 / algoParam.r % 不符合曲率约束，代价置为inf
    newNode.cost = inf;
else
    newNode.cost = nearestNode.cost + length;
end

end
