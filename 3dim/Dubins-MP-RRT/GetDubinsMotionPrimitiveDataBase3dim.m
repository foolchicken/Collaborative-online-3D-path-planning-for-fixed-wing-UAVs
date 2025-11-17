function mpDataBase3dim = GetDubinsMotionPrimitiveDataBase3dim(speed, gridresxy, gridresz, yawdisNum, pitchdisNum, databasePointNum, dynamicCons, savename)
% 计算dubins运动基元三维数据库，并行计算版本加速计算

yawdisNum = max(yawdisNum, 4);
if mod(pitchdisNum, 2) == 1
    pitchdisNum = pitchdisNum + 1; % 必须为偶数
end

n = sqrt(databasePointNum);

mpDataBase2dim = GetDubinsMotionPrimitiveDataBase2dim(speed, gridresxy, yawdisNum, databasePointNum, dynamicCons); % 先计算二维数据库

mpDataBase3dim = cell(n, n, n); % 运动基元数据库

parfor ind = 1 : n ^ 3
    mpDataBase3dim{ind} = singleloop(ind, speed, gridresxy, gridresz, pitchdisNum, dynamicCons, mpDataBase2dim);
end


if nargin == 8
    save([savename '.mat'], 'mpDataBase3dim');
end

end


function motionPrimitives = singleloop(ind, speed, gridresxy, gridresz, pitchdisNum, dynamicCons, mpDataBase2dim)

n = size(mpDataBase2dim, 1);
[ii, jj, kk] = ind2sub([n, n, n], ind);
point = [(ii - 1) * gridresxy, (jj - 1) * gridresxy, (kk - 1) * gridresz]; % 目标点坐标

if ii == 1 && jj == 1
    motionPrimitives = [];
    return
end
if point(3) > hypot(point(1), point(2)) * tan(dynamicCons.pitchmax) % 不满足角度约束直接跳过
    motionPrimitives = [];
    return
end

motionPrimitive.yaw0 = 0;
motionPrimitive.yawf = 0;
motionPrimitive.pitch0 = 0;
motionPrimitive.pitchf = 0;
motionPrimitive.cost = inf;
motionPrimitive.path = [];

nstate2dim = size(mpDataBase2dim{end, end}, 1); % 二维数据库一对点有多少种状态

motionPrimitives = repmat(motionPrimitive, nstate2dim * (pitchdisNum / 2 + 1) * (pitchdisNum + 1), 1); % 扩展到三维，多了起点的pitchdisNum+1种状态和终点的pitchdisNum / 2+1种状态

iter = 1;

pitchres = (dynamicCons.pitchmax - dynamicCons.pitchmin) / pitchdisNum; % 俯仰角分辨率

for i = 1 : nstate2dim

    mp2dim = mpDataBase2dim{ii, jj}(i);
    path2dim = mp2dim.path;
    path2dimLength = mp2dim.cost;

    for j = 0 : pitchdisNum
        pitch0 = dynamicCons.pitchmin + j * pitchres;
        for k = 0 : pitchdisNum / 2 % 注意pitchdisNum总是偶数，以保证两边平分
            pitchf = k * pitchres;

            motionPrimitive.yaw0 = mp2dim.theta0;
            motionPrimitive.pitch0 = pitch0;
            motionPrimitive.yawf = mp2dim.thetaf;
            motionPrimitive.pitchf = pitchf;
            motionPrimitive.cost = inf;
            motionPrimitive.path = [];

            if isinf(mp2dim.cost)
                motionPrimitives(iter) = motionPrimitive;
                iter = iter + 1;
                continue % 二维路径不可行跳过
            end

            dist2dim = vecnorm(diff(path2dim(:, 1 : 2)), 2, 2); % 相邻点之间的二维路径距离
            cumdist = cumsum(dist2dim);
            cumdist = [0; cumdist]; % 二维路径累加和

            vz0 = speed * tan(pitch0);
            vzf = speed * tan(pitchf);
            deltaV = vzf - vz0;

            tf = norm(point) / speed * (1 + abs(pitchf - pitch0) / 2); % 乘以放大系数保证安全
            t = tf * cumdist / path2dimLength;

            x = [tf ^ 3 / 12 -tf ^ 2 / 4; tf ^ 2 / 4 -tf / 2] \ [point(3) - vz0 * tf; deltaV];
            alpha = x(1);
            beta = x(2);
            z = alpha * t .^ 3 / 12 - beta * t .^ 2 / 4 + vz0 * t;

            path = [path2dim(:, 1 : 2) z]; % 实际存在误差

            dist3dim = vecnorm(diff(path), 2, 2); % 相邻点之间的三维路径距离
            length = sum(dist3dim);

            motionPrimitive.cost = length;
            motionPrimitive.path = DouglasPeucker(path, 10); % 抽稀保存，减少数据量

            motionPrimitives(iter) = motionPrimitive;
            iter = iter + 1;

        end
    end
end

end


