function dist = GetDist2Tree(pos, RRTree)
% 计算当前位置到RRT树所有节点的距离
% 如果节点不活跃或者不满足俯仰角约束则距离为inf

dist = zeros(size(RRTree)); % 采样点到RRT树的欧式距离

for i = 1 : size(RRTree, 1)
    if RRTree(i).ifact
        dist(i) = norm(RRTree(i).pos - pos); % 只计算活跃节点且符合俯仰角约束
    else
        dist(i) = inf;
    end
end

