function isvalid = PointCheck(point, algoParam)

% isvalid = true;
% return

for i = 1 : numel(algoParam.obs) % 先判断是否在障碍物内部
    if algoParam.obs(i).ifinObs(point)
        isvalid = false;
        return
    end
end

point = XY2LL(point, algoParam.start); % 再判断是否和地形碰撞
I = round((algoParam.mapRef.LatitudeLimits(2) - point(2)) / algoParam.mapRef.CellExtentInLatitude) + 1;
J = round((point(1) - algoParam.mapRef.LongitudeLimits(1)) / algoParam.mapRef.CellExtentInLongitude) + 1;
%[I, J] = geographicToDiscrete(algoParam.mapRef, point(2), point(1)); %
%将地理坐标转换为数组索引 matlab自带的太慢了
%[I1, J1] = geographicToDiscrete(algoParam.mapRef, point(2), point(1)); % 将地理坐标转换为数组索引
if I > size(algoParam.highData,1) || J > size(algoParam.highData,2) || I < 1 || J < 1
    isvalid = false;
    return
end

h = algoParam.highData(I, J);

isvalid = point(3) > h + algoParam.dsafe;


end
