close all
clear
clc

addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')
addpath('D:\Matlabproject\硕士毕业论文\2dim\Quick-Dubins-MP-RRT')
addpath('D:\Matlabproject\硕士毕业论文\3dim')
algoParam.maxIter = 500; % 最大扩展次数，即树的最大容量
algoParam.start = [108.851497597421	35.1322671226846 2000, deg2rad(0), deg2rad(0)]; % 经纬高、航向角（和x轴夹角）、俯仰角
algoParam.goal = [109.040782330388	35.1429152923380, 2000, deg2rad(90), deg2rad(0)];
algoParam.checkStep = 1;
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.dsafe = 100; % 最小离地高度
algoParam.accu = 0.005; % 收敛精度


dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.maxh = 4000; % 最大飞行高度

[pathXYH, pathLLH, allnodeList, algoParam] = QuickDubinsRRT3dim(algoParam, dynamicCons, "mpdatav60.mat", "mpdataFTAv60.mat",...
    'D:\MapService\西安北部地图small2.tif');

%% 绘图
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点
generateObs; % 生成障碍物
figure
geoaxes("Basemap", "satellite") % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
set(gca, 'TickLabelFormat', '-dd') % 十进制度，用减号 (-) 表示南纬和西经
hold on
geoplot(pathLLH(:, 2), pathLLH(:, 1), '-', 'LineWidth', 2, 'Color', 'w', "DisplayName", "path")
for i = 1 : numel(Obslist)
    Obslist(i).plot('Color', 'r');
end
hold off
legend
title('2dim path')

% figure
% axesm('MapProjection', 'gstereo', 'MapLatLimit', algoParam.mapRef.LatitudeLimits, 'MapLonLimit', algoParam.mapRef.LongitudeLimits, ...
%     'Grid', 'on', 'MeridianLabel', 'on', 'ParallelLabel', 'on') % 创建了一个基于轴的地图，使用极射赤平投影化为矩形
% geoshow(algoParam.highData, algoParam.mapRef, 'DisplayType', 'surface')
% tightmap; % 删除基于轴的贴图周围的空白
% demcmap(algoParam.highData)
% daspectm('m', 1);
% view(20, 35)
% camlight % 创建或移动光源
% % ax = gca;
% % ax.Box = 'off'; % 不显示边框
% % axis tight % 删除基于轴的贴图周围的空白
% setm(gca, 'MLabelLocation', 0.2, 'MLabelRound', -1)
% setm(gca, 'PLabelLocation', 0.2, 'PLabelRound', -1)
% setm(gca, 'MLabelParallel', 'south')
% setm(gca, 'PLabelMeridian', 'east')
% setm(gca, 'FLineWidth', 0.5)
% plot3m(pathLLH(:, 2), pathLLH(:, 1), pathLLH(:, 3), LineWidth = 1.5, Color = 'red')
% title('3dim MP-RRT route plan')

dist2dim = vecnorm(diff(pathXYH(:, 1 : 2)), 2, 2);
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist] / 1000;
hterrain = zeros(size(cumdist));
for i = 1 : numel(hterrain)
    [I, J] = geographicToDiscrete(algoParam.mapRef, pathLLH(i, 2), pathLLH(i, 1)); % 将地理坐标转换为数组索引
    hterrain(i) = algoParam.highData(I, J);
end
figure
hold on
plot(cumdist, pathXYH(:, 3), 'LineWidth', 1.5, 'DisplayName', '飞行高度')
plot(cumdist, hterrain, 'LineWidth', 1.5, 'DisplayName', '地面高度')
hold off
xlabel('横向距离 km')
ylabel('飞行高度 m')
title('飞行高度剖面')
legend

%[path, newnodeList] = PathHSmooth(allnodeList{10}, algoParam);

%% 航迹跟踪仿真
addpath('D:\Matlabproject\硕士毕业论文\航迹跟踪仿真')
dynamicCons = algoParam.dynamicCons;
L1 = 800;
TransitionRadius = 100;
speed = algoParam.speed;
initialState = [0 0 algoParam.start(3) speed, algoParam.start(4 : 5)]; % 初始状态，位置、速度、航向角、俯仰角
state = UAVPathFollowSimulate(pathXYH, initialState, L1, TransitionRadius, dynamicCons);

figure
plot3(pathXYH(:, 1), pathXYH(:, 2), pathXYH(:, 3), 'LineWidth', 1.5, 'DisplayName', '规划路径')
hold on
plot3(state(:, 1), state(:, 2), state(:, 3), 'LineWidth', 1.5, 'DisplayName', '飞行轨迹')
hold off
axis equal
grid on
legend

figure
hold on
plot(state(:,end), rad2deg(state(:,5)), 'LineWidth', 1.5, 'DisplayName', '偏航角')
plot(state(:,end), rad2deg(state(:,6)), 'LineWidth', 1.5, 'DisplayName', '俯仰角')
plot(state(:,end), rad2deg(state(:,7)), 'LineWidth', 1.5, 'DisplayName', '滚转角')
hold off
legend


