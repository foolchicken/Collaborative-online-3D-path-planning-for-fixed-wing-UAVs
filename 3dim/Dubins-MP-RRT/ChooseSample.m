function qs = ChooseSample(algoParam, bestLength)
% 对搜索空间进行采样

region = [algoParam.mapRef.LongitudeLimits; algoParam.mapRef.LatitudeLimits]; % 地图范围

if ~isinf(bestLength)  % 如果已经找到路径，在椭球中进行搜索
    p = [0, 0, 0];
    while true
        x = -1 + 2 * rand();
        y = -1 + 2 * rand();
        z = -1 + 2 * rand();
        if x * x + y * y  + z * z< 1
            p(1) = x; p(2) = y;p(3) = z;
            break
        end
    end
    % transform to ellipse
    goalxyh = LL2XY(algoParam.goal(1 : 3), algoParam.start);
    a = bestLength/2;
    c = norm(goalxyh) / 2;
    qs = transform(p, a, c, [0,0,algoParam.start(3)], goalxyh);
else
    if rand < 0.1
        qs = LL2XY(algoParam.goal(1 : 3), algoParam.start);
        return
    end
    while 1
        pos = [rand() * (region(1, 2) - region(1, 1)) + region(1), rand() * (region(2, 2) - region(2, 1)) + region(2, 1)];
        [I, J] = geographicToDiscrete(algoParam.mapRef, pos(2), pos(1));
        if ~isnan(I) && ~isnan(J)
            break
        end
    end
    ht = algoParam.highData(I, J);
    qs = [LL2XY(pos, algoParam.start), rand() * (algoParam.dynamicCons.maxh - ht) + ht];
end

% 连续采样转化为离散采样
qs = NormalizedGrid(qs, algoParam);

end


function pnew = transform(p, a, c, start, goal)
% 球体转换为椭球，输入二维球体(0,1)坐标，长轴长度a、焦点距离c
center = (goal - start)/2 + start;
pitch = atan2(goal(3)- start(3), norm(goal(1:2) - start(1:2)));
yaw = atan2(goal(2)-start(2), goal(1) - start(1));

Lyaw = [cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
Lpitch = [cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)];

b = sqrt(a * a - c * c);
p = p.*[a, b, b/10]; % 球到椭球
pnew = Lpitch*Lyaw*p'; % transform
pnew = pnew';

pnew = pnew + center;

end
