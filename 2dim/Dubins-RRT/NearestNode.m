function [nearestNode, path, dist] = NearestNode(sample, RRTree, algoParam)
% 从RRT树中寻找最近节点

dist = zeros(size(RRTree, 1), 1);
mindist = inf;
for i = 1 : size(RRTree, 1)
    %[pathseg, length] = DubinsPath(RRTree(i).pose, sample, algoParam.r);
    param = dubins_core(RRTree(i).pose, sample, algoParam.r);
    length = dubins_length(param);
    dist(i) = length;
    if length < mindist
        mindist = length;
        %path = pathseg;
        path = param;
        nearestNode = RRTree(i);
    end
end


end
