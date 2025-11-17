function pathLLH = XYZ2LLHPath(pathxyz, originPoint)
% 发射系坐标转化为经纬高坐标

pathLLH = zeros(size(pathxyz));
[pathLLH(:, 2), pathLLH(:, 1), pathLLH(:, 3)] = enu2geodetic(pathxyz(:, 1), pathxyz(:, 2), pathxyz(:, 3), ...
    originPoint(2), originPoint(1), originPoint(3), wgs84Ellipsoid); % XYZ反转回经纬高


end