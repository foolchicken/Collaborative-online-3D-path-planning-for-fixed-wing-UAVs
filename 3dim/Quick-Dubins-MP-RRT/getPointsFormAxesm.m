close all
clear
clc

[Z, R] = readgeoraster('D:\MapService\西安北部地图small2.tif');
%[Z, R] = readgeoraster('D:\MapService\downloadmap\9236_9638DEM\ASTGTM2_N36E092\ASTGTM2_N36E092_dem.tif');

distLon = distance(R.LatitudeLimits(1), R.LongitudeLimits(1), R.LatitudeLimits(1), R.LongitudeLimits(2), wgs84Ellipsoid);
distLat = distance(R.LatitudeLimits(1), R.LongitudeLimits(1), R.LatitudeLimits(2), R.LongitudeLimits(1), wgs84Ellipsoid);

disp(['经度方向距离：' num2str(distLon / 1000) ' km'])
disp(['纬度方向距离：' num2str(distLat / 1000) ' km'])

figure
geoaxes("Basemap", "satellite")
geolimits(R.LatitudeLimits, R.LongitudeLimits)
set(gca, 'TickLabelFormat', '-dd') % 十进制度，用减号 (-) 表示南纬和西经
hold on
area = [R.LongitudeLimits(1) R.LatitudeLimits(1);
    R.LongitudeLimits(1) R.LatitudeLimits(2);
    R.LongitudeLimits(2) R.LatitudeLimits(2);
    R.LongitudeLimits(2) R.LatitudeLimits(1)];
area = area(convhull(area), :);
for i = 1 : size(area, 1) - 1
    geoplot([area(i, 2) area(i + 1, 2)], [area(i, 1) area(i + 1, 1)], '-', 'LineWidth', 2, 'Color', 'w')
end

disp('输入s选择起点，输入g选择终点，输入c选择圆形障碍物，输入p选择多边形障碍物，输入q退出')
Obslist = [];
while 1
    k = waitforbuttonpress;
    if k == 1
        c = get(gcf, 'CurrentCharacter');
        switch c
            case 's'
                n = input('请输入起点数量:');
                start = ginput(n);
                for i = 1 : n
                    geoplot(start(i, 1), start(i, 2), 'o', 'Color', 'w')
                end
                start = start(:,2 : -1 : 1);
            case 'g'
                n = input('请输入终点数量:');
                goal = ginput(n);
                for i = 1 : n
                    geoplot(goal(i, 1), goal(i, 2), 'o', 'Color', 'w')
                end
                goal = goal(:,2 : -1 : 1);
            case 'c'
                center = ginput(1);
                center = center(2 : -1 : 1);
                r = input('请输入障碍物半径:');
                obs = Obs(center, r);
                obs.plot('Color', 'r');
                Obslist = [Obslist; obs];
            case 'p'
                n = input('请输入障碍物顶点数量:');
                plist = ginput(n);
                plist = plist(:, 2 : -1 : 1);
                obs = Obs(plist);
                obs.plot('Color', 'r');
                Obslist = [Obslist; obs];
            case 'q'
                break
        end
    end
end
