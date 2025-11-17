function isvalid = PointCheck(point, algoParam)

isvalid = true;
for i = 1 : numel(algoParam.obs) % 先判断是否在障碍物内部
    if algoParam.obs(i).ifinObs(point)
        isvalid = false;
        return
    end
end

% point = XY2LL(point, algoParam.start); % 再判断是否和地形碰撞
% [I, J] = geographicToDiscrete(algoParam.mapRef, point(2), point(1)); % 将地理坐标转换为数组索引
% if isnan(I) || isnan(J)
%     isvalid = false;
%     return
% end
% 
% h = algoParam.highData(I, J);
% 
% isvalid = point(3) > h + algoParam.dsafe;


end
