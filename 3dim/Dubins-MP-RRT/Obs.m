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
                isInside = norm(point(1:2) - obj.center) < obj.range;
            end
        end

        % 绘制障碍物
        function plot(obj, varargin)
            color = 'w';
            coortype = 1; % 坐标类型，1为LL，2为XY，默认为1

            for i = 1 : numel(varargin)-1
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
                    center = [0, 0];
                else
                    center = obj.center;
                end
                x = center(1) + obj.range * cos(theta);
                y = center(2) + obj.range * sin(theta);
                if coortype == 1
                    for i = 1 : numel(x)
                        pos(i, :) = XY2LL([x(i), y(i)], obj.center);
                    end
                    geoplot(pos(:, 2), pos(:, 1), 'LineWidth', 1.5, 'Color', color);
                else
                    plot(x, y, 'LineWidth', 1.5, 'Color', color);
                end
            end
        end
    end
end
