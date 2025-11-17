function endNode1 = Steer2Goal(newNode, RRTree, algoParam)
% 尝试向向目标点连线

nend1 = algoParam.goal(1 : 2) + algoParam.maxstep/2 * [cos(algoParam.goal(3) + pi) sin(algoParam.goal(3) + pi)];
nend2 = algoParam.goal(1 : 2) + algoParam.maxstep/2 * [cos(algoParam.goal(3)) sin(algoParam.goal(3))];

ntemp = RRTree(newNode.parent);
[path, length, maxk] = GetBspline2Order(ntemp.pos, newNode.pos, nend1);

endNode1.parent = newNode.ind;
endNode1.pos = nend1;
endNode1.path = path;

if isinf(maxk) || maxk > 1 / algoParam.r || ~PathCollisionCheck(path, algoParam) % 不符合曲率约束，或者路径不可行，代价置为inf
    endNode1.cost = inf;
else
    endNode1.cost = newNode.cost + length;
end

[path1, ~, maxk1] = GetBspline2Order(newNode.pos, nend1, nend2); % 检测从nearestNode和nend1的中点出发的样条曲线

if isinf(maxk1) || maxk1 > 1 / algoParam.r || ~PathCollisionCheck(path1, algoParam)% 不符合曲率约束，代价置为inf
    endNode1.cost = inf;
end


end