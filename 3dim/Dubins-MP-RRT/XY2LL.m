function point = XY2LL(point, orginPoint)
% xy坐标转换为经纬度坐标，距离越远误差越大，使用WGS84模型
% 10km误差在50m

a = 6378137; % 赤道半径（单位：米）
f = 1 / 298.257223563; % 扁率
b = a * (1 - f); % 极半径

% 计算当前纬度的地球半径
lat1 = orginPoint(2);
lon1 = orginPoint(1);
lat_rad = lat1 * (pi / 180);
R = (a * (1 - f) ^ 2) / ((1 - f * sin(lat_rad) ^ 2) ^ (3 / 2)); % 有效半径

% 计算新的纬度
delta_lat = point(2) / R; % 纬度弧度变化
lat2 = lat1 + (delta_lat * (180 / pi)); % 转换为度

% 计算新的经度
delta_lon = (point(1) / (R * cos(lat_rad))) * (180 / pi); % 弧度转为度
lon2 = lon1 + delta_lon;

point(1:2) = [lon2 lat2];

end
