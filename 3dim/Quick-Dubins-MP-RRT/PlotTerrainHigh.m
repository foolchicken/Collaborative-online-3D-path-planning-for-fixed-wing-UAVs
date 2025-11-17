function p = PlotTerrainHigh(pathXYH, pathLLH, color, filedir)
% 绘制地形高度剖面
[Z, R] = readgeoraster(filedir);

dist2dim = vecnorm(diff(pathXYH(:, 1 : 2)), 2, 2);
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist] / 1000;
hterrain = zeros(size(cumdist));
for i = 1 : numel(hterrain)
    I = round((R.LatitudeLimits(2) - pathLLH(i, 2)) / R.CellExtentInLatitude) + 1;
    J = round((pathLLH(i, 1) - R.LongitudeLimits(1)) / R.CellExtentInLongitude) + 1;
    hterrain(i) = Z(I, J);
end

p = plot(cumdist, hterrain, 'LineWidth', 1.5, 'Color', color, 'LineStyle','-');
