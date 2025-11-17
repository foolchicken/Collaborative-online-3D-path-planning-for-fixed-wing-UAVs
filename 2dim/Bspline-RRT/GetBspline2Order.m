function [path, length, maxk, kk] = GetBspline2Order(P1, P2, P3)
% 计算2次b样条曲线路径的长度和最大曲率
% 路径长度由曲线和曲线到两个端点的线段组成

%% 平移到P2为原点、P1->P2为x轴的坐标系
P1new = P1 - P2;
P3new = P3 - P2;
%P2new = [0, 0];

theta = pi - atan2(P1new(2), P1new(1));
rotaMat = [cos(theta) -sin(theta); sin(theta) cos(theta)];
P1_ = rotaMat * P1new';
P1new = P1_';

P3_ = rotaMat * P3new';
P3new = P3_';

%% 计算最大曲率
alpha = atan2(P3new(2), P3new(1));
L0 = hypot(P1new(1), P1new(2));
L1 = hypot(P3new(1), P3new(2));

if cos(alpha) > L1 / L0
    maxk = L0 * abs(sin(alpha)) / L1 ^ 2;
elseif cos(alpha) > L0 / L1
    maxk = L1 * abs(sin(alpha)) / L0 ^ 2;
else
    maxk = (L0 ^ 2 + L1 ^ 2 - 2 * L0 * L1 * cos(alpha)) ^ 1.5 / (L0 ^ 2 * L1 ^ 2 * sin(alpha) ^ 2);
end

maxk = sign(alpha)*maxk; % 考虑正负号

%disp(['最大曲率为 ' num2str(maxk)])

%% 计算长度
a = L0 ^ 2 + L1 ^ 2 - 2 * L0 * L1 * cos(alpha);
b = 2 * L0 * L1 * cos(alpha) - 2 * L0 ^ 2;
c = L0 ^ 2;

if a ~= 0
    splinelen = (4 * a * c - b ^ 2) * (log( (2 * a + b) / (2 * sqrt(a)) + sqrt(a + b + c) ) - log( b / (2 * sqrt(a)) + sqrt(c) ) ) / (8 * a ^ 1.5) + ...
        ((2 * a + b) * sqrt(a + b + c) - b * sqrt(c)) / (4 * a);
else
    splinelen = L0 / 2 + L1 / 2;
end

%disp(['长度为 ' num2str(S)])

%% 计算路径
r = 1 / maxk; % 转弯半径
nseg = max(splinelen / r * 4, 20); % 最少20个点
nseg = min(nseg, 50); % 最大50个点
t = 0 : 1 / nseg : 1;
t = t';
if t(end) ~= 1
    t = [t; 1];
end
splinepath = 0.5 * ((1 - 2 * t + t .^ 2) .* P1 + (1 + 2 * t - 2 * t .^ 2) .* P2 + t .^ 2 .* P3) ;


kk = t*L1*sin(alpha)./(0.5*( -(-2 + 2*t)*L0 + 2*t*L1*cos(alpha) )) - tan(theta);
% n1 = max(ceil(L0 / r * 4), 5);
% deltal = L0 / 2 / n1;
% path1 = zeros(n1, 2);
% for i = 0 : n1 - 1
%     path1(i + 1, :) = P1 + (splinepath(1,:) - P1) * deltal * i / (L0 / 2);
% end
%
% n2 = max(ceil(L1 / r * 4), 5);
% deltal = L1 / 2 / n2;
% path2 = zeros(n2, 2);
% for i = 1 : n2
%     path2(i, :) = splinepath(end,:) + (P3 - splinepath(end,:)) * deltal * i / (L1 / 2);
% end
%
% path = [path1; splinepath; path2];
% length = splinelen + L0 / 2 + L1 / 2;
path = splinepath;
length = splinelen;

end
