function point = LL2XY(point, orginPoint)
% 经纬坐标转换为xy坐标，距离越远误差越大，使用WGS84模型
% 10km误差在50m
% 虽然计算有误差，但是只要坐标也是通过XY2LL函数计算的可以抵消误差

a = 6378137; % 赤道半径（单位：米）
f = 1 / 298.257223563; % 扁率
b = a * (1 - f); % 极半径

% 计算当前纬度的地球半径
lat1 = orginPoint(2);
lon1 = orginPoint(1);
lat_rad = lat1 * (pi / 180);
R = (a * (1 - f) ^ 2) / ((1 - f * sin(lat_rad) ^ 2) ^ (3 / 2)); % 有效半径

% 计算y
y = deg2rad(point(2) - lat1)*R;

% 计算x
x = deg2rad(point(1) - lon1)*R*cos(lat_rad);

point(1:2) = [x y];


end