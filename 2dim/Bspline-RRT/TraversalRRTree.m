function RRTree = TraversalRRTree(RRTree, startind, fun)
% 对RRT树从指定索引开始遍历并对每个节点执行fun函数

queue = zeros(int32(size(RRTree, 1)), 1); % 队列遍历树，存储子节点索引
top = 0;
bottom = 1;
queue(bottom) = startind;

while top < bottom % 巧妙地遍历树
    top = top + 1;
    cur = queue(top);
    RRTree(cur) = fun(RRTree(cur));
    childs = [];
    for j = 1 : size(RRTree, 1)
        if RRTree(j).parent == cur
            childs = [j, childs];
        end
    end
    for k_ind = 1 : numel(childs)
        bottom = bottom + 1;
        queue(bottom) = childs(k_ind);
    end
end
