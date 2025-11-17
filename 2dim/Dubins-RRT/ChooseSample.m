function sample = ChooseSample(algoParam)
% 选择采样点，输入算法参数

if rand < 0.8 % 偏向目标的RRT
    sample = [rand(1,2) .* size(algoParam.map) * algoParam.resolutionMap, rand()*2*pi]; % 随机采样
else
    sample = algoParam.goal; % 向终点前进
end



end