function path = ArcSample(center, start, length, dir, deltaL)
% 内切离散圆弧，输入圆弧中心、起点、长度、左右转方向、每一段的最大长度

nseg = max(ceil(length / deltaL),2);
r = norm(center - start);
angle = length / r;
deltaAngle = angle / nseg;

alpha0 = atan2(start(2) - center(2), start(1) - center(1));

path = zeros(nseg + 1, 2);
path(1, :) = start;

alpha = alpha0;
if dir == 'R'
    for i = 1 : nseg
        alpha = alpha - deltaAngle;
        path(i + 1, :) = center + r * [cos(alpha) sin(alpha)];
    end
elseif dir == 'L'
    for i = 1 : nseg
        alpha = alpha + deltaAngle;
        path(i + 1, :) = center + r * [cos(alpha) sin(alpha)];
    end
end


end


