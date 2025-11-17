function [path, nodeList] = FindWayBack(RRTree, algoParam)
% 回溯寻找路径

goalxyh = LL2XY(algoParam.goal, algoParam.start);
%% 寻找RRT树的终点
dist2end = zeros(size(RRTree, 1), 1); % 树中所有节点到终点的欧氏距离
for i = 1 : size(RRTree, 1)
    dist2end(i) = norm(RRTree(i).state(1:3) - goalxyh(1 : 3));
end
[mindist, ind] = min(dist2end);
if mindist > 1
    warning('未找到路径！')
end

nodeList = GetParentNodeList(RRTree, ind);
path = [];

for i=1:numel(nodeList)
    path = [path; nodeList(i).path(1:end-1,:)];
end
path = [path; goalxyh(1:3)];


end
