% 类测试
close all
clear
clc

o1 = Obs([109, 34.1], 5000);
o2 = Obs([108.8, 34.3; 108.8, 34.5; 109, 34.5; 109, 34.3]);

figure
geoaxes("Basemap", "satellite", "ZoomLevel", 11) % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
hold on
o1.plot('Color', 'w');
o2.plot('Color', 'r');
hold off

orgin = [109, 34.1];
o1 = o1.LL2XY(orgin);
o2 = o2.LL2XY(orgin);

figure
hold on
o1.plot('CoorType', 'XY', 'Color', 'k');
o2.plot('CoorType', 'XY', 'Color', 'r');
hold off
axis equal


