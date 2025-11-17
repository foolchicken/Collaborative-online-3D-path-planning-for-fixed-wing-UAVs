function nodes = NearNodes(qs, RRTree, range, algoParam)
% 寻找在运动基元数据库覆盖范围内的邻节点
% 输入采样节点、RRT树、基元数据库范围


% iter = size(RRTree, 1);
% ner = 10 * range * ( log(iter + 1) / iter ) ^ (1 / 2); % 计算椭球半径的公式，根据迭代次数和维度动态计算
% r = min(ner, range);

nodes = [];
for i = 1 : size(RRTree, 1)
    pos = RRTree(i).pos;
    if RRTree(i).ifact && abs(pos(1) - qs(1)) <= range && abs(pos(2) - qs(2)) <= range  % 寻找活跃且在数据库采样范围内的点
        nodes = [nodes; RRTree(i)];
    end
end

end
