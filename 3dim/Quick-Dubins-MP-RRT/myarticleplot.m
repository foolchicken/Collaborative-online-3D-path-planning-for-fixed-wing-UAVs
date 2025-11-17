close all
clear
clc

load("scene5path2.mat")

addpath('D:\Matlabproject\硕士毕业论文\航迹跟踪仿真')
addpath('D:\Matlabproject\硕士毕业论文\论文插图')
minR = algoParam.speed ^ 2 / (9.8 * tan(algoParam.dynamicCons.rollmax));
WritePath2txt(allPathLLH, 'D:\VSproject\VGMissionPlanPlatform203article\uavobspath.txt', 1);
L1 = 1000;
TransitionRadius = 200;
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
% WritePath2txt(allTrajectory, 'D:\VSproject\SDK2.0.11_test_scenario\scenario\4机避障轨迹.txt', 2);
% 
% for i = 1 : numel(Obslist)
%     Obslist(i).writePos2txt(4000, 'D:\VSproject\SDK2.0.11_test_scenario\scenario\obspos.txt');
% end
% 
% 
% I = imread('协同避障场景2推演软件.png');
% 
% figure('Position', [600, 200, size(I, 2), size(I, 1)]);
% image(I);
% hold on;
% p1 = plot(nan, nan, 'r-', 'DisplayName', '线条 1', 'LineWidth', 1.5); % 红线
% p2 = plot(nan, nan, 'g-', 'DisplayName', '线条 2', 'LineWidth', 1.5); % 绿线
% p3 = plot(nan, nan, 'b-', 'DisplayName', '线条 2', 'LineWidth', 1.5); % 蓝线
% p4 = plot(nan, nan, 'Color', [0, 0.8275, 1], 'DisplayName', '线条 2', 'LineWidth', 1.5); % 蓝线
% % 添加图例
% legend('uav1', 'uav2', 'uav3', 'uav4', 'FontSize', 12, 'Location', 'northeast')
% legend('boxoff')
% legend('TextColor', 'white')
% xticks([]); % 隐藏 x 轴刻度
% yticks([]); % 隐藏 y 轴刻度
% tightfig;
% %set(gca, 'Position', [0.1, 0.1, 0.8, 0.8]);  % 例如，左边和下边各 15%，右边和上边各 15%
% %saveas(gcf, 'D:\硕士毕业论文\素材\第五章\蚁群算法场景2规划软件.svg'); % gcf: 获取当前图形窗口

addpath("D:\Matlabproject\地形图绘制\slanCM\")

figure
ShowTIFmap2('D:\MapService\沙漠地图第五章低精度.tif')
hold on
for i = 1 : numel(Obslist)
    Obslist(i).plot3('Color', 'r', 'Centerh', 4000);
end
zlim([3000, 20000])


% clc; clear; close all;
% 
% % 生成一个不规则的多边形底面（7 边形）
% numSides = 6;  % 多边形边数
% theta = linspace(0, 2*pi, numSides); % 角度
% r = 2 + 0.5 * rand(1, numSides); % 不规则半径
% x_base = r .* cos(theta); % X 坐标
% y_base = r .* sin(theta); % Y 坐标
% z_base = zeros(size(x_base)); % 底面 Z 坐标
% z_top = ones(size(x_base)) * 5; % 顶面 Z 坐标（柱体高度 5）
% 
% % 绘制底面
% figure; hold on; grid on;
% patch(x_base, y_base, z_base, 'r', 'FaceAlpha', 0.6);
% 
% % 绘制顶面
% patch(x_base, y_base, z_top, 'b', 'FaceAlpha', 0.6);
% 
% % 绘制侧面
% for i = 1:numSides
%     X = [x_base(i), x_base(i+1), x_base(i+1), x_base(i)];
%     Y = [y_base(i), y_base(i+1), y_base(i+1), y_base(i)];
%     Z = [z_base(i), z_base(i+1), z_top(i+1), z_top(i)];
%     patch(X, Y, Z, 'g', 'FaceAlpha', 0.6);
% end
% 
% % 轴设置
% axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('不规则多边形柱体');
% view(3); % 3D 视角

