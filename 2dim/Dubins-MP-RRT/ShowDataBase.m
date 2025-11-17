function ShowDataBase(mpDataBase)
% 展示数据库

figure
hold on
for i = 1 : size(mpDataBase, 1)
    for j = 1 : size(mpDataBase, 2)
        if isempty(mpDataBase{i, j}), continue; end
        for k = 1 : size(mpDataBase{i, j}, 1)
            if isinf(mpDataBase{i, j}(k).cost), continue; end
            path = interpolate(mpDataBase{i, j}(k).pathseg.obj, linspace(0, mpDataBase{i, j}(k).pathseg.Length, 30));
            plot(path(:, 1), path(:, 2), 'LineWidth', 1);
        end
    end
end
hold off
axis equal
grid on
