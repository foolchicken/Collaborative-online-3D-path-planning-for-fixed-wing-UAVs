function newNode = Steer(sample, nearestNode, algoParam)
% 从nearestNode出发向sample连线

param = dubins_core(nearestNode.pose, sample, algoParam.r);
length = dubins_length(param);

nseg = max(length / algoParam.r * 4, 5); % 路径分割的线段数量
path = dubins_path_sample_many(param, length / nseg);
if norm(path(end, 1 : 2) - sample(1 : 2)) > 1e-1
    path = [path; sample];
end

newNode.parent = nearestNode.ind;
newNode.pose = sample;
newNode.path = path;
newNode.cost = nearestNode.cost + length;


end
