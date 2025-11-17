function path = FindWayBack(RRTree, algoParam)
% 回溯寻找路径
nend1 = algoParam.goal(1 : 2) + algoParam.maxstep/2 * [cos(algoParam.goal(3) + pi) sin(algoParam.goal(3) + pi)];
nend2 = algoParam.goal(1 : 2) + algoParam.maxstep/2 * [cos(algoParam.goal(3)) sin(algoParam.goal(3))];

%% 寻找RRT树的终点
dist2end = inf*ones(size(RRTree, 1), 1); % 树中所有节点到nend1的欧氏距离
for i = 3 : size(RRTree, 1)
    if isempty(RRTree(i).path)
        break
    end
    dist2end(i) = norm(RRTree(i).pos - nend1);
end
[mindist, ind] = min(dist2end);
if mindist > 1
    warning('未找到路径！')
end

nearestNode = RRTree(ind);

%% 回溯寻找所有路径
path = [];
child = nearestNode;
parent = child.parent;
while parent > 0
    path = [child.path; path];
    child = RRTree(parent);
    parent = child.parent;
end
pathend = GetBspline2Order(RRTree(RRTree(ind).parent).pos, nend1, nend2);
path = [path; pathend];

end
