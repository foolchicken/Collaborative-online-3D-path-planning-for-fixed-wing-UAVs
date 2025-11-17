function nodeList = PathVerticalOptimize(nodeList, algoParam)
% 对节点的高度和俯仰角进行优化

nodeh = zeros(size(nodeList));
nodehterrain = nodeh;
cumdistxy = nodeh;
for i = 1 : size(nodeList, 1)
    nodeh(i) = nodeList(i).pos(3);
    nodehterrain(i) = nodeList(i).hterrain;
end


for i = 2 : size(nodeList, 1)
    cumdistxy(i) = cumdistxy(i - 1) + sum(vecnorm(diff(nodeList(i).path(:, 1 : 2)), 2, 2 )) / 1000; % 单位km
end
% figure
% plot(cumdistxy, nodeh)
% hold on

itermax = 20;
kc = 1; % 弹性力系数（越大效果越明显，但是过大可能发生震荡！）
kr = 100; % 斥力系数
alpha = 0.1;

algoParam.obs = [];
for iter = 1 : itermax
    nodehold = nodeh;
    for i = 2 : size(nodeh, 1) - 1
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
        frontstate(3) = nodeh(i - 1);
        newstate(3) = nodehold(i) + deltah;
        nextstate(3) = nodeh(i + 1);
        path1 = Get3dimDubinsPath(frontstate, newstate, algoParam.speed, algoParam.dynamicCons);
        path2 = Get3dimDubinsPath(newstate, nextstate, algoParam.speed, algoParam.dynamicCons);

        if PathCollisionCheck(path1, algoParam) && PathCollisionCheck(path2, algoParam) % 如果高度优化后的路径可行
            nodeh(i) = nodehold(i) + deltah;
        end
        [path1, ~, ~, pitchf] = Get3dimDubinsPath(frontstate, newstate, algoParam.speed, algoParam.dynamicCons, false);
        newstate(5) = pitchf;
        path2 = Get3dimDubinsPath(newstate, nextstate, algoParam.speed, algoParam.dynamicCons);
        if PathCollisionCheck(path1, algoParam) && PathCollisionCheck(path2, algoParam) % 对俯仰角进一步优化
            nodeList(i).state(5) = pitchf;
        end
    end
    %     plot(cumdistxy, nodeh)
    %     pause(0.5)
end

for i = 2 : numel(nodeList) - 1
    nodeList(i).pos(3) = nodeh(i);
    nodeList(i).state(3) = nodeh(i);
end

% for i = 2 : numel(nodeList)
%     pathseg = Get3dimDubinsPath(nodeList(i - 1).state, nodeList(i).state, algoParam.speed, algoParam.dynamicCons, true);
%     if ~PathCollisionCheck(pathseg, algoParam)
%         warning('路径碰撞')
%     end
% end
% 

