function [path, newlength, info] = AdjDubinsPathLength(start_pos, start_heading, end_pos, end_heading, R, deltaLength, dir, ifplot)
% 绕飞法调整dubins路径长度
% 输出航路点、新长度和额外信息

if nargin == 7
    ifplot = false;
end

[pathseg, length] = DubinsPath([start_pos start_heading], [end_pos end_heading], R);
if deltaLength == 0
    nseg = max(length / R * 4, 5); % 路径分割的线段数量
    path = interpolate(pathseg.obj, linspace(0, pathseg.Length, nseg));
    path = path(:, 1 : 2);
    info = 0;
    newlength = length;
    return
end

rotateAntiClock = @(theta)[cos(theta) -sin(theta); sin(theta) cos(theta)]; % 逆时针旋转的变换矩阵
rotateClock = @(theta)[cos(theta) sin(theta); -sin(theta) cos(theta)]; % 顺时针旋转的变换矩阵

if deltaLength < 2 * pi * R % 绕飞式调整长度
    a = pathseg.MotionLengths(2) / 2;
    f1 = @(b)(sqrt(a ^ 2 + b ^ 2 - 4 * b * R) + ...
        2 * R * (atan2(2 * R, sqrt(a ^ 2 + b ^ 2 - 4 * b * R) ) + atan2(b - 2 * R, a)) - ...
        a - deltaLength / 2);

    bmax = sqrt((a + deltaLength / 2) ^ 2 - a ^ 2);
    if a < 2 * R
        deltaLengthMax = 4 * R * (pi / 2 - atan(sqrt(4 * R ^ 2 - a ^ 2) / a)) - pathseg.MotionLengths(2);
        if deltaLengthMax < deltaLength
            %warning("直线段长度不够调整")
            b = 2 * R - sqrt(4 * R ^ 2 - a ^ 2);
            info = -1;
        else
            b = fzero(f1, bmax * 0.7);
            info = 1;
        end
    else
        b = fzero(f1, bmax);
        info = 1;
    end

    middle = (pathseg.p1 + pathseg.p2) / 2; % 直线段中点
    vec1 = middle - pathseg.p1;
    if dir == 'R'
        vec2 = rotateClock(pi / 2) * vec1';
    elseif dir == 'L'
        vec2 = rotateAntiClock(pi / 2) * vec1';
    end
    vec2 = vec2';
    vec2 = vec2 * b / norm(vec2);
    extendP = middle + vec2; % 扩展点
    extendPdir = atan2(vec1(2), vec1(1)); % 扩展点处的航向角

    [pathsegNew1, length1] = DubinsPath([pathseg.p1 extendPdir], [extendP extendPdir], R);
    [pathsegNew2, length2] = DubinsPath([extendP extendPdir], [pathseg.p2 extendPdir], R);

    startCircle = ArcSample(pathseg.circleStart, start_pos, pathseg.MotionLengths(1), pathseg.MotionTypes{1}, R / 4);
    goalCircle = ArcSample(pathseg.circleGoal, pathseg.p2, pathseg.MotionLengths(3), pathseg.MotionTypes{3}, R / 4);
    nseg = max(length1 / R * 4, 5); % 路径分割的线段数量
    path1 = interpolate(pathsegNew1.obj, linspace(0, pathsegNew1.Length, nseg));
    nseg = max(length2 / R * 4, 5); % 路径分割的线段数量
    path2 = interpolate(pathsegNew2.obj, linspace(0, pathsegNew2.Length, nseg));
    path = [startCircle(1 : end - 1, :); path1(:, 1 : 2); path2(2 : end, 1 : 2); goalCircle(2 : end, :)]; % 路径拼接
    newlength = pathseg.MotionLengths(1) + pathseg.MotionLengths(3) + length1 + length2;
    if ifplot
        show(pathseg.obj)
        hold on
        show(pathsegNew1.obj)
        show(pathsegNew2.obj)
        scatter(pathseg.circleStart(1), pathseg.circleStart(2))
        scatter(pathseg.circleGoal(1), pathseg.circleGoal(2))
        scatter(extendP(1), extendP(2))
        axis equal
        grid on
    end
else % 盘旋式调整长度
    newlength = length + deltaLength;
    n = floor(deltaLength / (2 * pi * R)); % 盘旋圈数
    hoverR = deltaLength / (2 * pi * n); % 盘旋半径
    nseg = max(length / R * 4, 5); % 路径分割的线段数量
    path = interpolate(pathseg.obj, linspace(0, pathseg.Length, nseg));
    path = path(:, 1 : 2);
    info = 2;
    middle = (pathseg.p1 + pathseg.p2) / 2; % 直线段中点
    vec1 = middle - pathseg.p1;
    if dir == 'R'
        vec2 = rotateClock(pi / 2) * vec1';
    elseif dir == 'L'
        vec2 = rotateAntiClock(pi / 2) * vec1';
    end
    vec2 = vec2';
    vec2 = vec2 * hoverR / norm(vec2);
    hoverCenter = middle + vec2; % 扩展点
    singleHoverPath = ArcSample(hoverCenter, middle, 2 * pi * hoverR, dir, R / 5);
    [~, insertind] = min(vecnorm(path - middle, 2, 2)); % 插入点的索引
    hoverPath = [];
    for i = 1 : n
        hoverPath = [hoverPath; singleHoverPath];
    end
    path = [path(1 : insertind - 1, :); hoverPath; path(insertind + 1 : end, :)];
end

end

