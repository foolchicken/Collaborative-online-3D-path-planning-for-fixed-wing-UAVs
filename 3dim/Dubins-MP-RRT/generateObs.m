% 生成障碍物信息

%Obslist(1) = Obs([109.06 34.75; 109.06 34.8; 109.2 34.8; 109.2 34.75]);
Obslist(1) = Obs([109, 34.75], 3000);
Obslist(2) = Obs([109.10, 34.7], 2500);
Obslist(3) = Obs([108.95, 34.84], 3500);

% figure
% geoaxes("Basemap", "satellite", "ZoomLevel", 11) % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
% hold on
% for i = 1 : numel(Obslist)
%     Obslist(i).plot('Color', 'w');
% end
% hold off
