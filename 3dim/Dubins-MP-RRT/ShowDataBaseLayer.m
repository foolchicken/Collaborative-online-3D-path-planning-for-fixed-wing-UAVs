function ShowDataBaseLayer(mpDataBase)
% 展示数据库的其中一层

figure
for i = 1 : size(mpDataBase, 1)
    for j = 1 : size(mpDataBase, 2)
        if isempty(mpDataBase{i, j}), continue; end
        ifscatter = false;
        for k = 1 : size(mpDataBase{i, j}, 1)
            if isinf(mpDataBase{i, j}(k).cost), continue; end
            path = mpDataBase{i, j}(k).path;
            plot3(path(:, 1), path(:, 2), path(:, 3), 'LineWidth', 1);
            hold on
            if ~ifscatter
                scatter3(path(end, 1), path(end, 2), path(end, 3))
                ifscatter = true;
            end
        end
    end
end
hold off
axis equal
grid on
