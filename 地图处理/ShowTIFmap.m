function ShowTIFmap(filedir, tick, dim)
% 绘制数字高程地图

[Z, R] = readgeoraster(filedir);
Z = double(Z);
if nargin == 2
    dim = 3;
end

%% 以世界地图的方式展示

% figure
% latlim = R.LatitudeLimits;
% lonlim = R.LongitudeLimits;
% worldmap(latlim,lonlim) % 创建了一个基于轴的世界地图
% meshm(Z, R)
% demcmap(Z) % 根据高程数据来设置颜色图
Z = Z-min(min(Z));

%% 将图片三维化
axesm('MapProjection', 'gstereo', 'MapLatLimit', R.LatitudeLimits, 'MapLonLimit', R.LongitudeLimits, ...
    'Frame', 'on', 'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on')
geoshow(Z, R, 'DisplayType', 'surface')
axis tight
demcmap(Z)
axis off
if dim == 3
    daspectm('m', 6);
    view(20, 35)
end
camlight % 创建或移动光源

setm(gca, 'MLabelLocation', tick, 'MLabelRound', -1, 'MLabelParallel','south')
setm(gca, 'PLabelLocation', tick, 'PLabelRound', -1, 'PLabelMeridian','east')
%setm(gca, 'PLabelLocation', tick, 'PLabelRound', -1)


end
