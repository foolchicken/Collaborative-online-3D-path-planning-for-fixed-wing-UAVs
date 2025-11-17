% 第五章仿真，导入到任务规划软件版本
close all
clear
clc

addpath('D:\Matlabproject\硕士毕业论文\Dubins绕飞')
addpath('D:\Matlabproject\硕士毕业论文\2dim\Quick-Dubins-MP-RRT')
addpath('D:\Matlabproject\硕士毕业论文\mpdata\dubins')

nuav = 3;

% initialState = [92.2659516602544	36.0656942697201 5000, deg2rad(90), deg2rad(0);
%     92.5625098050252	36.0267757737348, 5200, deg2rad(90), deg2rad(0);
%     92.8474382186286	36.0985902100100, 5350, deg2rad(90), deg2rad(0)];
%
% endState = [92.4462124933504	36.5226464517915, 4500, deg2rad(90), deg2rad(0);
%     92.5072685819797	36.5179733585571, 4500, deg2rad(90), deg2rad(0);
%     92.5944915657358	36.5203099404542, 4500, deg2rad(90), deg2rad(0)];
initialState = [92.9201240384253	36.3628633185142 5200, deg2rad(160), deg2rad(0);
    92.77684505322	36.1643643351178, 5700, deg2rad(120), deg2rad(0);
    92.6264733264464	36.0868432399384, 5350, deg2rad(90), deg2rad(0)];

endState = [92.1147651550771	36.67002680029535, 4300, deg2rad(180), deg2rad(0);
    92.1147651550771	36.63471599834374, 4300, deg2rad(180), deg2rad(0);
    92.1147651550771	36.5973775485585, 4300, deg2rad(180), deg2rad(0)];

algoParam.maxIter = 500; % 最大扩展次数，即树的最大容量
algoParam.checkStep = 1;
algoParam.maxFailedAttempts = 100; % 节点最大扩展失败次数
algoParam.dsafe = 500; % 最小离地高度
algoParam.accu = 0.005; % 收敛精度

dynamicCons.rollmin = -deg2rad(30); % 滚转角
dynamicCons.rollmax = deg2rad(30);
dynamicCons.pitchmin = -deg2rad(30); % 俯仰角
dynamicCons.pitchmax = deg2rad(30);
dynamicCons.maxh = 7000; % 最大飞行高度

for i = 1 : nuav
    algoParam.start = initialState(i, :); % 经纬高、航向角（和x轴夹角）、俯仰角
    algoParam.goal = endState(i, :);
    [pathXYH, pathLLH, nodeList, algoParam] = QuickDubinsRRT3dim(algoParam, dynamicCons, "mpdatav140.mat", "mpdataFTAv140.mat", ...
        'D:\MapService\downloadmap\9236_9638DEM\ASTGTM2_N36E092\ASTGTM2_N36E092_dem.tif');
    uavPath(i).pathXYH = pathXYH;
    uavPath(i).pathLLH = pathLLH;
    uavPath(i).nodeList = nodeList;
end

tic
[allPathXYH, allPathLLH] = AdjUAVpathLength(uavPath, algoParam, 'D:\MapService\downloadmap\9236_9638DEM\ASTGTM2_N36E092\ASTGTM2_N36E092_dem.tif');
toc
for i = 1 : nuav
    len(i) = sum(vecnorm(diff(allPathXYH{i}), 2, 2));
end
%% 绘图
color = ["red", "yellow", "blue", "white"];
legendName = cell(1, nuav);
goalxyh = LL2XY(algoParam.goal, algoParam.start); % xyz坐标的终点
%generateObs; % 生成障碍物
figure
geoaxes("Basemap", "satellite") % 创建一个地理坐标区，背景为卫星地图，缩放级别12，数字越大分辨率越小
set(gca, 'TickLabelFormat', '-dd') % 十进制度，用减号 (-) 表示南纬和西经
hold on
for i = 1 : nuav
    legendName{i} = "uav " + num2str(i);
    g(i) = geoplot(allPathLLH{i}(:, 2), allPathLLH{i}(:, 1), '-', 'LineWidth', 1.5, 'Color', color(i));
end
for i = 1 : numel(Obslist)
    Obslist(i).plot('Color', 'r');
end
hold off
legend(g, legendName, 'FontSize', 10)
legend('boxoff')
legend('TextColor', 'white')
legend('FontSize', 10)

for i = 1 : nuav
    figure
    hold on
    PlotPathHigh(allPathXYH{i}, 'red');
    PlotTerrainHigh(allPathXYH{i}, allPathLLH{i}, 'green', 'D:\MapService\downloadmap\9236_9638DEM\ASTGTM2_N36E092\ASTGTM2_N36E092_dem.tif');
    xlabel('航迹距离/km')
    ylabel('飞行高度/m')
    hold off
    legend('飞行高度', '地形高度', 'Location', 'southwest', 'FontSize', 8)
    set(gcf, 'Units', 'centimeters');
    set(gcf, 'Position', [10, 10, 8, 5]); % 这里设置图窗的宽度为8厘米，高度为6厘米
end

%linestyle = ["-",":","-.","--"];
linestyle = ["-", "-", "-", "-"];
figure
ShowTIFmap('D:\MapService\沙漠地图第五章低精度.tif', 0.2)
hold on
for i = 1 : nuav
    g(i) = plot3m(allPathLLH{i}(:, 2), allPathLLH{i}(:, 1), allPathLLH{i}(:, 3) - 855, '-', 'LineWidth', 1.5, 'Color', color(i), 'LineStyle', linestyle(i));
end
hold off
legend(g, legendName, 'FontSize', 10)

%% 导出和三自由度仿真
addpath('D:\Matlabproject\硕士毕业论文\航迹跟踪仿真')
addpath('D:\Matlabproject\硕士毕业论文\论文插图')
WritePath2txt(allPathLLH, 'D:\VSproject\VGMissionPlanPlatform203article\uavobspath.txt', 1);

L1 = 2000;
TransitionRadius = 600;
speed = algoParam.speed;

allTrajectory = cell(nuav, 1);
for i = 1 : nuav
    start = [0 0 initialState(i, 3) speed, initialState(i, 4 : 5)]; % 初始状态，位置、速度、航向角、俯仰角
    state = UAVPathFollowSimulate(allPathXYH{i}, start, L1, TransitionRadius, dynamicCons);
    disp(['无人机 ' num2str(i) ' 飞行时长' num2str(state(end, end)) ' s'])
    allTrajectory{i} = state(1 : 2 : end, 1 : 3);
    for j = 1 : size(allTrajectory{i}, 1)
        allTrajectory{i}(j, :) = XY2LL(allTrajectory{i}(j, :), initialState(i, 1 : 2));
    end
    figure
    %     plot3(allPathXYH{i}(:, 1), allPathXYH{i}(:, 2), allPathXYH{i}(:, 3), 'LineWidth', 1.5, 'DisplayName', '规划路径')
    %     hold on
    %     plot3(state(:, 1), state(:, 2), state(:, 3), 'LineWidth', 1.5, 'DisplayName', '飞行轨迹')
    plot(allPathXYH{i}(:, 1), allPathXYH{i}(:, 2), 'LineWidth', 1.5, 'DisplayName', '规划路径')
    hold on
    plot(state(:, 1), state(:, 2), 'LineWidth', 1.5, 'DisplayName', '飞行轨迹')
    hold off
    axis equal
    grid on
    legend

    %         figure
    %         hold on
    %         plot(state(:, end), rad2deg(state(:, 5)), 'LineWidth', 1.5, 'DisplayName', '偏航角')
    %         plot(state(:, end), -rad2deg(state(:, 6)), 'LineWidth', 1.5, 'DisplayName', '俯仰角')
    %         plot(state(:, end), rad2deg(state(:, 7)), 'LineWidth', 1.5, 'DisplayName', '滚转角')
    %         hold off
    %         legend
end
WritePath2txt(allTrajectory, 'D:\VSproject\SDK2.0.11_test_scenario\scenario\避障轨迹.txt', 2);
allTrajectory136 = allTrajectory;
for i = 1 : nuav
    allTrajectory136{i} = allTrajectory136{i}(1 : 1369, :);
end
WritePath2txt(allTrajectory136, 'D:\VSproject\SDK2.0.11_test_scenario\scenario\避障轨迹136s.txt', 2);

for i = 1 : numel(Obslist)
    Obslist(i).writePos2txt(4000, 'D:\VSproject\SDK2.0.11_test_scenario\scenario\obspos.txt');
end

%[~,ind] = min(vecnorm(allTrajectory{3}(:,1:2) - [92.3472 36.2764],2,2))

XY2LL([-200, 0], allTrajectory{3}(end, 1 : 2))

% tic;
% sum = 0;
% for i = 1:1000000
%     sum = sum + i;
% end
% toc;

