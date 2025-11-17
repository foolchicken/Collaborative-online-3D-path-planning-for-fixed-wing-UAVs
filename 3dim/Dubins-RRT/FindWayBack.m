function path = FindWayBack(RRTree, algoParam)
% 回溯寻找最优路径

%% 寻找RRT树的终点
originPoint = algoParam.start;
[goal(1), goal(2), goal(3)] = geodetic2enu(algoParam.goal(2), algoParam.goal(1), algoParam.goal(3), originPoint(2), originPoint(1), originPoint(3), wgs84Ellipsoid);
[nearest2end, dist2end] = NearestNode([goal, algoParam.goal(4:end)] , RRTree, algoParam); % 所有节点到终点的距离
mindist = min(dist2end);
if mindist > algoParam.maxstep / 10
    warning('未找到路径！')
end

%% 回溯寻找所有路径
path = [];
child = nearest2end;
parent = child.parent;
while parent > 0
    %path = [interpolate(child.path.obj, 0 : algoParam.checkStep / 5 : child.path.Length); path];
    path = [child.path; path];
    child = RRTree(parent);
    parent = child.parent;
end


end