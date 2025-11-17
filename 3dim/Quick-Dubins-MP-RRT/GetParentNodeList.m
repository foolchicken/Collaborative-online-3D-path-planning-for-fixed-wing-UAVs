function nodeList = GetParentNodeList(RRTree, ind)
% 获取RRT树第ind个节点的父节点列表

nodeList = [];
child = RRTree(ind);
parent = child.parent;
while parent > 0
    nodeList = [child; nodeList];
    child = RRTree(parent);
    parent = child.parent;
end
nodeList = [RRTree(1); nodeList];


