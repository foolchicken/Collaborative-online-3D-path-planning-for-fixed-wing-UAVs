function [nearestNode, dist] = NearestNode(sample, RRTree, algoParam)
% 从RRT树中寻找最近节点，并返回最近路径和所有节点的三维dubins距离

nearestNode = [];
dist = zeros(size(RRTree, 1), 1);
mindist = inf;
minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));
for i = 1 : size(RRTree, 1)
    dubinsParam = dubins_core([sample(1 : 2) pi / 2 - sample(4)], [RRTree(i).pose(1 : 2) pi / 2 - RRTree(i).pose(4)], minR);
    path2dimLength = dubins_length(dubinsParam); % 二维dubins路径长度
    deltaZ = sample(3) - RRTree(i).pose(3); % 高度变化量
    %length = hypot(path2dimLength, deltaZ); % 路径长度使用平方和近似
    length = path2dimLength;
    dist(i) = length;
    if length < mindist
        mindist = length;
        nearestNode = RRTree(i);
    end
end


end
