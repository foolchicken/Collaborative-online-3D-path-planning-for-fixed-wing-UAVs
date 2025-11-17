% 2次b样条曲线曲率、长度计算测试
close all
clear
clc

%P = randi([0, 5], 3, 2);
P = [101.2018  118.3542; 96.3035  137.7451; 99.5806  157.4748];
%P = [-10 0;0 0;10.5 10.5];

% n = 1000;
% t = zeros(n, 1);
% for i = 1 : n
%     tic
%     [path, length, maxk] = GetBspline2Order(P(1, :), P(2, :), P(3, :));
%     t(i) = toc;
% end
% disp(['2次B样条曲线计算轨迹平均用时' num2str(sum(t) / n * 1000) '毫秒'])
% 
% figure
% hold on
% scatter(P(:, 1), P(:, 2))
% plot(path(:, 1), path(:, 2), 'LineWidth', 1.5)
% hold off
% axis equal

[path, length, maxk1] = GetBspline2Order(P(1,:), P(2,:), P(3,:))

dt = 0.1;
t = 0 : dt : 1;
t = t';
c = 0.5 * ((1 - 2 * t + t .^ 2) .* P(1, :) + (1 + 2 * t - 2 * t .^ 2) .* P(2, :) + t .^ 2 .* P(3, :));

% 实践证明论文提出的B样条属于均匀b样条曲线，样条曲线起终点为线段中点，并相切于线段
figure
hold on
scatter(P(:, 1), P(:, 2))
plot(c(:, 1), c(:, 2), 'LineWidth', 1.5)
scatter(c(1, 1), c(1, 2))
scatter(c(end, 1), c(end, 2))
hold off
axis equal
title('2次均匀b样条曲线')

% 平移到P2为原点、P1->P2为x轴的坐标系
P(1, :) = P(1, :) - P(2, :);
P(3, :) = P(3, :) - P(2, :);
P(2, :) = [0, 0];

theta = pi - atan2(P(1, 2), P(1, 1));
rotaMat = [cos(theta) -sin(theta); sin(theta) cos(theta)];
P1 = rotaMat * P(1, :)';
P(1, :) = P1';

P3 = rotaMat * P(3, :)';
P(3, :) = P3';

c = 0.5 * ((1 - 2 * t + t .^ 2) .* P(1, :) + (1 + 2 * t + t .^ 2) .* P(2, :) + t .^ 2 .* P(3, :));
% 
% figure
% hold on
% scatter(P(:, 1), P(:, 2))
% plot(c(:, 1), c(:, 2), 'LineWidth', 1.5)
% scatter(c(1, 1), c(1, 2))
% scatter(c(end, 1), c(end, 2))
% hold off
% axis equal
% title('旋转后的2次均匀b样条曲线')


alpha = atan2(P3(2), P3(1));
L0 = hypot(P(1, 1), P(1, 2));
L1 = hypot(P(3, 1), P(3, 2));

if cos(alpha) > L1 / L0
    maxk = L0 * abs(sin(alpha)) / L1 ^ 2;
elseif cos(alpha) > L0 / L1
    maxk = L1 * abs(sin(alpha)) / L0 ^ 2;
else
    maxk = (L0 ^ 2 + L1 ^ 2 - 2 * L0 * L1 * cos(alpha)) ^ 1.5 / (L0 ^ 2 * L1 ^ 2 * sin(alpha) ^ 2);
end

disp(['最大曲率为 ' num2str(maxk)])
disp(['最小转弯半径为 ' num2str(1/maxk)])

a = L0 ^ 2 + L1 ^ 2 - 2 * L0 * L1 * cos(alpha);
b = 2 * L0 * L1 * cos(alpha) - 2 * L0 ^ 2;
c = L0 ^ 2;

if a ~= 0
    S = (4 * a * c - b ^ 2) * (log( (2 * a + b) / (2 * sqrt(a)) + sqrt(a + b + c) ) - log( b / (2 * sqrt(a)) + sqrt(c) ) ) / (8 * a ^ 1.5) + ...
        ((2 * a + b) * sqrt(a + b + c) - b * sqrt(c)) / (4 * a);
else
    S = L0 / 2 + L1 / 2;
end

disp(['长度为 ' num2str(S)])



