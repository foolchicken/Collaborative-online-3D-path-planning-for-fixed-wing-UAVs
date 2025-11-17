function isvalid = PointCheck(point, algoParam)

[I, J] = geographicToDiscrete(algoParam.mapRef, point(2), point(1)); % 将地理坐标转换为数组索引
if isnan(I) || isnan(J)
    isvalid = false;
    return
end
h = algoParam.highData(I, J);

isvalid = point(3) > h;


end