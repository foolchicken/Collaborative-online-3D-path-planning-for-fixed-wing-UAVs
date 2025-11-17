function path = FindWayBack(RRTree, algoParam)
% 回溯寻找路径

%% 寻找RRT树的终点
dist2end = zeros(size(RRTree, 1), 1); % 树中所有节点到终点的欧氏距离
for i = 1 : size(RRTree, 1)
    dist2end(i) = norm(RRTree(i).pose(1 : 2) - algoParam.goal(1 : 2));
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


end
