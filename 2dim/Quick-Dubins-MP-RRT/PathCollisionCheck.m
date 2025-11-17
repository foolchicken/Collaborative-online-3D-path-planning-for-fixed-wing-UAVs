function isvalid = PathCollisionCheck(path, algoParam)
% 路径碰撞检查，true为可行路径
% isvalid = true;
% return

isvalid = false;

mapSize = size(algoParam.map); % 栅格地图大小
% 实际地图范围
xmin = 0;
ymin = 0;
xmax = (mapSize(2) - 1) * algoParam.resolutionMap;
ymax = (mapSize(1) - 1) * algoParam.resolutionMap;

if (any(path(:, 1) > xmax))
    return;
elseif (any(path(:, 1) < xmin))
    return;
elseif (any(path(:, 2) > ymax))
    return;
elseif (any(path(:, 2) < ymin))
    return;
end


indx = round((path(:, 1) - xmin) / algoParam.resolutionMap + 1);
indy = round((path(:, 2) - ymin) / algoParam.resolutionMap + 1);
ind = sub2ind(mapSize, indy, indx); % x坐标是数组的列
if algoParam.map(ind(end)) == 0 % 栅格为0表示不可行
    return;
end

%% 检测路径上的点
ind = ind(1 : algoParam.checkStep : numel(ind));
if (any(algoParam.map(ind) == 0))
    return;
end

%% 检测路径右侧
indxr = round((path(:, 1) + algoParam.dsafe - xmin) / algoParam.resolutionMap + 1);
indxr(indxr > mapSize(2)) = mapSize(2);
ind = sub2ind(mapSize, indy, indxr); % x坐标是数组的列
ind = ind(1 : algoParam.checkStep : numel(ind)); % 检测中间的点
if (any(algoParam.map(ind) == 0))
    return;
end

%% 检测路径左侧
indxl = round((path(:, 1) - algoParam.dsafe - xmin) / algoParam.resolutionMap + 1);
indxl(indxl < 1) = 1;
ind = sub2ind(mapSize, indy, indxl); % x坐标是数组的列
ind = ind(1 : algoParam.checkStep : numel(ind)); % 检测中间的点
if (any(algoParam.map(ind) == 0))
    return;
end

%% 检测路径上侧
indyu = round((path(:, 2) + algoParam.dsafe - ymin) / algoParam.resolutionMap + 1);
indyu(indyu > mapSize(1)) = mapSize(1);
ind = sub2ind(mapSize, indyu, indxr); % x坐标是数组的列
ind = ind(1 : algoParam.checkStep : numel(ind)); % 检测中间的点
if (any(algoParam.map(ind) == 0))
    return;
end

%% 检测路径下侧
indyd = round((path(:, 2) - algoParam.dsafe - ymin) / algoParam.resolutionMap + 1);
indyd(indyd < 1) = 1;
ind = sub2ind(mapSize, indyd, indxr); % x坐标是数组的列
ind = ind(1 : algoParam.checkStep : numel(ind)); % 检测中间的点
if (any(algoParam.map(ind) == 0))
    return;
end

isvalid = true;

end

