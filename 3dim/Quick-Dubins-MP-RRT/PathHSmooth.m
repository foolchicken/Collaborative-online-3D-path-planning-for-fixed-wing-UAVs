function [path, nodeList] = PathHSmooth(nodeList, algoParam)
% 高度方向的航迹平滑
% 使用梯度下降法

nodeh = zeros(size(nodeList));
nodehterrain = zeros(size(nodeList));
for i = 1 : size(nodeList, 1)
    nodeh(i) = nodeList(i).pos(3);
    nodehterrain(i) = nodeList(i).hterrain;
end

cumdistxy = 0;
for i = 2 : size(nodeList, 1)
    cumdistxy(i) = cumdistxy(i - 1) + sum(vecnorm(diff(nodeList(i).path(:, 1 : 2)), 2, 2 )) / 1000;
end
% figure
% plot(cumdistxy, nodeh)
% hold on
% plot(cumdistxy, nodehterrain)

nodeh1 = nodeh;
itermax = 40;
kc = 1; % 弹性力系数
kr = 100; % 斥力系数
alpha = 0.1;

algoParam.obs = [];
for iter = 1 : itermax
    nodehold = nodeh1;
    for i = 2 : size(nodeh1, 1) - 1
        fc = kc * ((nodehold(i - 1) - nodehold(i)) / (cumdistxy(i) - cumdistxy(i - 1)) + ...
            (nodehold(i + 1, :) - nodehold(i, :)) / (cumdistxy(i + 1) - cumdistxy(i)));
        d2terrain = nodehold(i) - nodehterrain(i);
        if d2terrain < algoParam.dsafe
            fr = 100;
        else
            fr = kr / (d2terrain - algoParam.dsafe);
        end
        deltah = alpha * (fc + fr);
        if deltah < -50, deltah = -50; elseif deltah > 50, deltah = 50; end % 限幅
        if abs(deltah) < 1
            continue;
        end
        frontstate = nodeList(i - 1).state;
        newstate = nodeList(i).state;
        nextstate = nodeList(i + 1).state;
        frontstate(3) = nodeh1(i-1);
        newstate(3) = nodehold(i) + deltah;
        nextstate(3) = nodeh1(i+1);
        path1 = Get3dimDubinsPath(frontstate, newstate, algoParam.speed, algoParam.dynamicCons);
        path2 = Get3dimDubinsPath(newstate, nextstate, algoParam.speed, algoParam.dynamicCons);

        if PathCollisionCheck(path1, algoParam) && PathCollisionCheck(path2, algoParam)
            nodeh1(i) = nodehold(i) + deltah;
        end
        [path1, ~, ~, pitchf] = Get3dimDubinsPath(frontstate, newstate, algoParam.speed, algoParam.dynamicCons, false);
        newstate(5) = pitchf;
        path2 = Get3dimDubinsPath(newstate, nextstate, algoParam.speed, algoParam.dynamicCons);
        if PathCollisionCheck(path1, algoParam) && PathCollisionCheck(path2, algoParam)
            nodeList(i).state(5) = pitchf;
        end
    end
%         plot(cumdistxy, nodeh1)
%         pause(0.5)
end

path = [];
for i = 2 : numel(nodeList)
    nodeList(i).pos(3) = nodeh1(i);
    nodeList(i).state(3) = nodeh1(i);

    [pathseg, length, ~, pitchf] = Get3dimDubinsPath(nodeList(i - 1).state, nodeList(i).state, algoParam.speed, algoParam.dynamicCons, true);
%     if ~PathCollisionCheck(pathseg, algoParam)
%         warning('路径碰撞')
%     end
    nodeList(i).path = pathseg;
    nodeList(i).cost = nodeList(i - 1).cost + length;
    nodeList(i).state(5) = pitchf;

    path = [path; pathseg];
end










