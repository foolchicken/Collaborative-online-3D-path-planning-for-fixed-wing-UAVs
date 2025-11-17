function node = ObsRangeCheck(node, obs, range)
% 检测节点下一次扩展时收影响的障碍物范围
node.obs = [];
for i = 1 : numel(obs)
    if norm(node.state(1:2) - obs(i).center) <= (1.1*range + obs(i).range)
        node.obs = [node.obs i];
    end
end

