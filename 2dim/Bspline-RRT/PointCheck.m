function if_feasible = PointCheck(point, map, res)
% 判断一个点是否可行，输入位置、地图、栅格地图分辨率
% 注意坐标对应地图索引是(Y,X)


mapSize = size(map);
xmin = 0;
ymin = 0;
xmax = mapSize(2) * res;
ymax = mapSize(1) * res;

if point(1) < 0 || point(1) > xmax || point(2) < 0 || point(2) > ymax
    if_feasible = false;
    return
end

indx = floor((point(1) - xmin) / res + 1);
indy = floor((point(2) - ymin) / res + 1);
if map(indy, indx)==1
    if_feasible = true;
else
    if_feasible = false;
end
