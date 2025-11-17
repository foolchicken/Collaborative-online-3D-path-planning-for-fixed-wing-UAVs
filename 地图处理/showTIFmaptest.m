% 展示数字高程地图
close all
clear
clc

load topo60c.mat
load pathLLH.mat

figure
imshow(topo60c, []);
title('原始tif影像');
axis xy

lat = 0 : 60;
lon = 90 * ones(size(lat));

%% 二维展示

figure
latlim = topo60cR.LatitudeLimits;
lonlim = topo60cR.LongitudeLimits;
worldmap(latlim, lonlim) % 创建了一个基于轴的世界地图
meshm(topo60c, topo60cR)
plotm(lat, lon, LineWidth = 1.5, Color = 'white')
demcmap(topo60c) % 根据高程数据来设置颜色图

figure
axesm('MapProjection', 'gstereo', 'MapLatLimit', topo60cR.LatitudeLimits, 'MapLonLimit', topo60cR.LongitudeLimits, ...
    'Frame', 'on', 'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on')
%geoshow(topo60c, topo60cR, 'DisplayType', 'surface')
meshm(topo60c, topo60cR, topo60cR.RasterSize, topo60c)
demcmap(topo60c)
axis off
plot3m(lat, lon, 4000*ones(size(lat)), LineWidth = 1.5, Color = 'white') % 即使是二维图，要想让线条展示也需要高度

lon = 45 * ones(size(lat));
z = 2000 * ones(size(lat));

[I, J] = geographicToDiscrete(topo60cR, lat, lon);
for i = 1 : numel(I)
    h(i) = topo60c(I(i), J(i));
end

%% 将图片三维化
figure
axesm('MapProjection', 'gstereo', 'MapLatLimit', topo60cR.LatitudeLimits, 'MapLonLimit', topo60cR.LongitudeLimits, ...
    'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on') % 创建了一个基于轴的地图，使用极射赤平投影化为矩形
meshm(topo60c, topo60cR, topo60cR.RasterSize, topo60c) % 在alt指定的高度显示规则曲面图
%tightmap; % 删除基于轴的贴图周围的空白
demcmap(topo60c)
daspectm('m', 200);
view(20, 35)
camlight % 创建或移动光源
ax = gca;
ax.Box = 'off'; % 不显示边框
axis tight % 删除基于轴的贴图周围的空白

plot3m(lat, lon, z, LineWidth = 1.5, Color = 'white')


lat = 38.7 : 0.01 : 39.3;
lon = 101.2 * ones(size(lat));
z = 3000 * ones(size(lon));


%framem('FlineWidth',4,'FEdgeColor','red')



figure
ShowTIFmap('D:\MapService\西安高程地图small.tif')
setm(gca, 'MLabelLocation', 0.2, 'MLabelRound', -1, 'MLabelParallel','south')
setm(gca, 'PLabelLocation', 0.2, 'PLabelRound', -1)
% plotm(pathLLH(:,2), pathLLH(:,1), LineWidth=1.5,Color='white')
% plotm(lat, lon, LineWidth=1.5,Color='white')




