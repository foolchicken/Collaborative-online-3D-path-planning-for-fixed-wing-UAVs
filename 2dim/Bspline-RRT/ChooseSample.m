function qs = ChooseSample(algoParam, bestLength)
% 对搜索空间进行采样

region = size(algoParam.map) * algoParam.resolutionMap; % 地图范围

if ~isinf(bestLength)  % 如果已经找到路径，在椭球中进行搜索
    while true
        p = [0, 0, 1];
        while true
            x = -1 + 2 * rand();
            y = -1 + 2 * rand();
            if x * x + y * y < 1
                p(1) = x; p(2) = y;
                break
            end
        end
        % transform to ellipse
        c = norm(algoParam.start(1 : 2) - algoParam.goal(1 : 2)) / 2;
        p = transform(bestLength / 2, c, algoParam.start(1 : 2), algoParam.goal(1 : 2)) * p';
        if p(1) >= 0 && p(1) <= region(1) && p(2) >= 0 && p(2) <= region(2)
            break
        end
    end
    pos = p(1:2)';

else
    if rand < 0.1
        qs = algoParam.goal(1:2);
        return
    end
    pos = [rand() * region(1), rand() * region(2)];

end

% 连续采样转化为离散采样
% qs = NormalizedGrid(pos, algoParam);
qs = pos;
end


function T = transform(a, c, start, goal)
% 球体转换为椭球，输入长轴长度a、焦点距离c
% center
center_x = (start(1) + goal(1)) / 2;
center_y = (start(2) + goal(2)) / 2;

% rotation
theta = -atan2(goal(2) - start(2), goal(1) - start(1));

% transform
b = sqrt(a * a - c * c);
T = [ a * cos(theta), b * sin(theta), center_x; ...
    -a * sin(theta), b * cos(theta), center_y; ...
    0, 0, 1];
end
