classdef Obs
    properties
        vertexs = [];
        center = [];
        range = [];
        type = [];
    end
    methods
        % 构造函数，根据输入变量数量初始化
        function obj = Obs(varargin)
            switch nargin
                case 1
                    obj.vertexs = varargin{1};
                    obj.type = ObsType.Poly;
                    k = convhull(obj.vertexs);
                    obj.vertexs = obj.vertexs(k, :); % 转为凸包后首尾相连
                    x = obj.vertexs(:, 1);
                    y = obj.vertexs(:, 2);
                    A = 0.5 * sum(x(1 : end - 1) .* y(2 : end) - x(2 : end) .* y(1 : end - 1));

                    % 计算重心
                    center(1) = (1 / (6 * A)) * sum((x(1 : end - 1) + x(2 : end)) .* (x(1 : end - 1) .* y(2 : end) - x(2 : end) .* y(1 : end - 1)));
                    center(2) = (1 / (6 * A)) * sum((y(1 : end - 1) + y(2 : end)) .* (x(1 : end - 1) .* y(2 : end) - x(2 : end) .* y(1 : end - 1)));
                    range = 0;
                    for i = 1 : size(obj.vertexs, 1) - 1
                        p = LL2XY(obj.vertexs(i, :), center);
                        if norm(p) > range
                            range = norm(p);
                        end
                    end
                    obj.center = center;
                    obj.range = range;
                case 2
                    obj.center = varargin{1};
                    obj.range = varargin{2};
                    obj.type = ObsType.Circle;
                otherwise
                    error('Invalid number of input arguments');
            end
        end

        % 坐标由经纬度转化为xy
        function obj = LL2XY(obj, orginPoint)
            if obj.type == ObsType.Poly
                for i = 1 : size(obj.vertexs, 1)
                    obj.vertexs(i, :) = LL2XY(obj.vertexs(i, :), orginPoint);
                end
            else
                obj.center = LL2XY(obj.center, orginPoint);
            end
        end

        % 坐标由xy转化为经纬度
        function obj = XY2LL(obj, orginPoint)
            for i = 1 : size(obj.vertexs, 1)
                obj.vertexs(i, :) = XY2LL(obj.vertexs(i, :), orginPoint);
            end
            obj.center = XY2LL(obj.center, orginPoint);
        end

        % 判断是否在障碍物内部
        function isInside = ifinObs(obj, point)
            if obj.type == ObsType.Poly
                isInside = inpolygon(point(1), point(2), obj.vertexs(:, 1), obj.vertexs(:, 2));
            else
                isInside = norm(point(1 : 2) - obj.center) < obj.range;
            end
        end

        % 返回障碍物中心
        function center = getCenter(obj)
            center = obj.center;
        end

        % 返回障碍物半径（范围）
        function R = getRange(obj)
            R = obj.range;
        end

        % 绘制障碍物
        function plot(obj, varargin)
            color = 'w';
            coortype = 1; % 坐标类型，1为LL，2为XY，默认为1

            for i = 1 : numel(varargin) - 1
                if strcmp(varargin{i}, 'CoorType')
                    if strcmp(varargin{i + 1}, 'LL')
                        coortype = 1;
                    elseif strcmp(varargin{i + 1}, 'XY')
                        coortype = 2;
                    else
                        error('无效的坐标类型！')
                    end
                elseif strcmp(varargin{i}, 'Color')
                    color = varargin{i + 1};
                end
            end

            if obj.type == ObsType.Poly
                for i = 1 : size(obj.vertexs) - 1
                    if coortype == 1
                        geoplot([obj.vertexs(i, 2) obj.vertexs(i + 1, 2)], [obj.vertexs(i, 1) obj.vertexs(i + 1, 1)], 'LineWidth', 1.5, 'Color', color);
                    else
                        plot([obj.vertexs(i, 1) obj.vertexs(i + 1, 1)], [obj.vertexs(i, 2) obj.vertexs(i + 1, 2)], 'LineWidth', 1.5, 'Color', color);
                    end
                end
            else
                theta = linspace(0, 2 * pi, 50);
                if coortype == 1
                    c = [0, 0];
                else
                    c = obj.center;
                end
                x = c(1) + obj.range * cos(theta);
                y = c(2) + obj.range * sin(theta);
                if coortype == 1
                    for i = 1 : numel(x)
                        pos(i, :) = XY2LL([x(i), y(i)], obj.center);
                    end
                    geoplot(pos(:, 2), pos(:, 1), 'LineWidth', 1.5, 'Color', color);
                    %fillm(lat_circle, lon_circle, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'none');

                    %geoscatter(obj.center(2), obj.center(1), obj.range/2, 'r', 'filled', 'MarkerFaceAlpha', 0.5); % 填充红色半透明
                else
                    plot(x, y, 'LineWidth', 1.5, 'Color', color);
                    %fill(x, y, 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % 填充红色半透明
                end
            end
        end
        function plot3(obj, varargin)
            color = 'r';
            centerh = 0;
            for i = 1 : numel(varargin) - 1
                if strcmp(varargin{i}, 'Color')
                    color = varargin{i + 1};
                elseif strcmp(varargin{i}, 'Centerh')
                    centerh = varargin{i + 1};
                end
            end

            if obj.type == ObsType.Poly
                xbase = obj.vertexs(1 : end - 1, 1);
                ybase = obj.vertexs(1 : end - 1, 2);
                ztop = 6000 * ones(numel(xbase), 1); % 顶面 Z 坐标（柱体高度）
                zbase = 4000*ones(size(ztop)); % 底面 Z 坐标
                patch(xbase, ybase, zbase, color, 'FaceAlpha', 0.6); % 绘制顶面和底面
                patch(xbase, ybase, ztop, color, 'FaceAlpha', 0.6);
                % 绘制侧面
                for i = 1 : numel(xbase)
                    next = i + 1;
                    if next > numel(xbase) ,next = 1;end
                    X = [xbase(i), xbase(next), xbase(next), xbase(i)];
                    Y = [ybase(i), ybase(next), ybase(next), ybase(i)];
                    Z = [zbase(i), zbase(next), ztop(next), ztop(i)];
                    patch(X, Y, Z, color, 'FaceAlpha', 0.6);
                end
            else
                [x, y, z] = sphere();
                x = x * obj.range;
                y = y * obj.range;
                z = z * obj.range;
                z(z<0) = 0;

                [lat, lon, ~] = enu2geodetic(x, y, z, obj.center(2), obj.center(1), 0, wgs84Ellipsoid); % XYZ反转回经纬高

                surf(lon, lat, z + centerh);
                %shading interp
            end
        end
        function writePos2txt(obj, high, dir)
            fileID = fopen(dir, 'a');
            if obj.type == ObsType.Poly
                for i = 1 : size(obj.vertexs) - 1
                    fprintf(fileID, '%f,', obj.vertexs(i, 1));
                    fprintf(fileID, '%f,', obj.vertexs(i, 2));
                    fprintf(fileID, '%f;', high);
                end
                fprintf(fileID, '\n');
            else
                theta = linspace(0, 2 * pi, 50);
                c = [0, 0];
                x = c(1) + obj.range * cos(theta);
                y = c(2) + obj.range * sin(theta);
                for i = 1 : numel(x)
                    pos = XY2LL([x(i), y(i)], obj.center);
                    fprintf(fileID, '%f ', pos(1));
                    fprintf(fileID, '%f ', pos(2));
                    fprintf(fileID, '%f,', high);
                end
                fprintf(fileID, '\n');
            end
            fclose(fileID);
        end
    end
end
