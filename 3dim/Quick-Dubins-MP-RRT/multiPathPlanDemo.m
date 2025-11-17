close all
clear
clc

addpath('D:\Matlabproject\硕士毕业论文\Dubins绕飞')
addpath('D:\Matlabproject\硕士毕业论文\2dim\Quick-Dubins-MP-RRT')
addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')

nuav = 4;
% 设置起点和终点 经纬高、航向角（和x轴夹角）、俯仰角
% initialState = [103.666501, 39.552998, 2000, deg2rad(90), deg2rad(0);
%     103.665, 39.56, 2000, deg2rad(90), deg2rad(0);
%     103.68, 39.55, 2000, deg2rad(0), deg2rad(0);
%     103.68, 39.58, 2000, deg2rad(45), deg2rad(0)];
% 
% endState = [103.735289, 39.579995, 2000, deg2rad(0), deg2rad(0);
%     103.735289, 39.579995, 2000, deg2rad(180), deg2rad(0);
%     103.735289, 39.579995, 2000, deg2rad(90), deg2rad(0);
%     103.735289, 39.579995, 2000, deg2rad(-90), deg2rad(0)];
initialState = [108.851497597421	35.1322671226846 1500, deg2rad(0), deg2rad(0);
    108.863935491506	35.1757588537967, 1500, deg2rad(-20), deg2rad(0);
    108.901106209460	35.2439749249822, 1650, deg2rad(-90), deg2rad(0);
    108.979450645764	35.2551830396129, 1550, deg2rad(0), deg2rad(0)];

endState = [109.038865039318	35.1340238285939, 1250, deg2rad(80), deg2rad(0);
    109.029908832398	35.1444830972941, 1250, deg2rad(-10), deg2rad(0);
    109.042699621458	35.1518067560821, 1250, deg2rad(-100), deg2rad(0);
    109.051655828378	35.1413474873819, 1250, deg2rad(170), deg2rad(0)];

% center = [109.040782330388	35.1429152923380];
% p1 = XY2LL(1000*[cosd(80) sind(80)], center);
% p2 = XY2LL(1000*[cosd(-10) sind(-10)], center);
% p3 = XY2LL(1000*[cosd(-100) sind(-100)], center);
% p4 = XY2LL(1000*[cosd(170) sind(170)], center);

algoParam.maxIter = 500; % 最大扩展次数，即树的最大容量
algoParam.checkStep = 1;
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.dsafe = 100; % 最小离地高度
algoParam.accu = 0.005; % 收敛精度

dynamicCons.rollmin = -deg2rad(15); % 滚转角
dynamicCons.rollmax = deg2rad(15);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.maxh = 3000; % 最大飞行高度

for i = 1 : nuav
    algoParam.start = initialState(i, :); % 经纬高、航向角（和x轴夹角）、俯仰角
    algoParam.goal = endState(i, :);
    [pathXYH, pathLLH, nodeList, algoParam] = QuickDubinsRRT3dim(algoParam, dynamicCons, "mpdatav60.mat", "mpdataFTAv60.mat",'D:\MapService\西安北部地图small2.tif');
    uavPath(i).pathXYH = pathXYH;
    uavPath(i).pathLLH = pathLLH;
    uavPath(i).nodeList = nodeList;
end

tic
[allPathXYH, allPathLLH] = AdjUAVpathLength(uavPath, algoParam,'D:\MapService\西安北部地图small2.tif');
toc
for i = 1 : nuav
    len(i) = sum(vecnorm(diff(allPathXYH{i}), 2, 2));
end
%% 绘图
color = ["red", "green", "cyan", "white"];
linestyle = ["-",":","-.","--"];
legendName = cell(1,nuav);
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点
generateObs; % 生成障碍物
figure
geoaxes("Basemap", "satellite") % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
set(gca, 'TickLabelFormat', '-dd') % 十进制度，用减号 (-) 表示南纬和西经
hold on
for i = 1 : nuav
    legendName{i} = "uav " + num2str(i);
    g(i) = geoplot(allPathLLH{i}(:, 2), allPathLLH{i}(:, 1), '-', 'LineWidth', 1.5, 'Color', color(i), 'LineStyle',linestyle(i));
end
for i = 1 : numel(Obslist)
    Obslist(i).plot('Color', 'r');
end
hold off
legend(g, legendName, 'FontSize',10)
legend('boxoff')
legend('TextColor','white')
legend('FontSize',10)
%title('2dim path')

figure
ShowTIFmap('D:\MapService\西安北部地图small2.tif', 0.05)
hold on
for i = 1 : nuav
    g(i) = plot3m(allPathLLH{i}(:, 2), allPathLLH{i}(:, 1), allPathLLH{i}(:, 3)-855, '-', 'LineWidth', 1.5, 'Color', color(i),'LineStyle',linestyle(i));
end
hold off
legend(g, legendName, 'FontSize',10)

color = ["red", "green", "blue", "cyan"];
% figure
% tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight');
% for i = 1 : nuav
%     nexttile
%     hold on
%     PlotPathHigh(allPathXYH{i}, color{1});
%     PlotTerrainHigh(allPathXYH{i}, allPathLLH{i}, color{2}, 'D:\MapService\西安北部地图small2.tif');
%     xlabel('横向距离 km')
%     ylabel('飞行高度 m')
%     title(['无人机' num2str(i) '高度剖面'])
%     hold off
%     legend('飞行高度', '地形高度', 'Location','southwest')
% end

for i = 1 : nuav
    figure
    hold on
    PlotPathHigh(allPathXYH{i}, 'red');
    PlotTerrainHigh(allPathXYH{i}, allPathLLH{i}, 'green', 'D:\MapService\西安北部地图small2.tif');
    xlabel('航迹距离/km')
    ylabel('高度/m')
    hold off
    legend('飞行高度', '地形高度', 'Location','southwest', 'FontSize', 8)
    set(gcf, 'Units', 'centimeters');
    set(gcf, 'Position', [10, 10, 8, 5]); % 这里设置图窗的宽度为8厘米，高度为6厘米
end

