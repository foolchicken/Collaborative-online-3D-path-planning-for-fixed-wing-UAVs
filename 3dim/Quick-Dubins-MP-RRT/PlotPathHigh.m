function p = PlotPathHigh(pathXYH, color)
% 绘制高度剖面

dist2dim = vecnorm(diff(pathXYH(:, 1 : 2)), 2, 2);
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist] / 1000;
p = plot(cumdist, pathXYH(:, 3), 'Color',color, 'LineWidth', 1.5, 'LineStyle','-.');



