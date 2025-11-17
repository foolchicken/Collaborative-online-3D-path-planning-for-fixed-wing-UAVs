function pos = NormalizedGrid(pos, algoParam)
% 将位置栅格规范化，只取运动基元栅格的位置

pos = floor((pos - algoParam.start(1 : 2)) / algoParam.resolutionMP) * algoParam.resolutionMP + algoParam.start(1 : 2);

end