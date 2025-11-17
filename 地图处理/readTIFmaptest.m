close all
clear
clc
% 读取 GeoTIFF 格式的 DEM 数据
[Z, R] = readgeoraster('D:\MapService\small1.tif');
[m, n, numOfColorLayer] = size(Z);

% 检查数据的有效性
if isempty(Z)
    error('无法读取 DEM 数据，请检查文件路径和文件格式。');
end

figure
imshow(Z,[]);
title('不带地理坐标的tif影像');


lat = 37; % 纬度
lon = 97; % 经度
% 将地理坐标转换为数组索引
[I,J] = geographicToDiscrete(R,lat,lon);

% load topo60c.mat
% 
% % slope为坡度，aspect为坡向（默认为角度制）
% % gradN,gradE为梯度的北向和东向分量
% [aspect,slope,gradN,gradE] = gradientm(topo60c,topo60cR);
% 
% figure;
% subplot(1, 2, 1);
% imagesc(aspect);
% title('Aspect (方向)');
% colorbar;
% 
% subplot(1, 2, 2);
% imagesc(slope);
% title('Slope (斜率)');
% colorbar;


