function ShowTIFmap2(filedir)
% 普通坐标系下绘制地形，上色依赖slanCM脚本

[Z, R] = readgeoraster(filedir);

lon = R.LongitudeLimits(1)+R.CellExtentInLongitude/2:R.CellExtentInLongitude:R.LongitudeLimits(2)-R.CellExtentInLongitude/2;
lat = R.LatitudeLimits(1)+R.CellExtentInLatitude/2:R.CellExtentInLatitude:R.LatitudeLimits(2)-R.CellExtentInLatitude/2;

[meshLon,meshLat] = meshgrid(lon, lat);
meshLat = meshLat(end:-1:1,:);

surf(meshLon, meshLat, Z)
shading interp
colormap(slanCM('terrain'))
xlabel('经度')
ylabel('纬度')
zlabel('高度')

