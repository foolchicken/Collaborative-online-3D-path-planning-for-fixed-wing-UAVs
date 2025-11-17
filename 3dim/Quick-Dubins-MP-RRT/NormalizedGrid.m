function pos = NormalizedGrid(pos, algoParam)
% 将xy位置进行栅格规范化，只取运动基元栅格的位置

pos(1:2) = round(pos(1:2)/ algoParam.resolutionMP) * algoParam.resolutionMP;

end