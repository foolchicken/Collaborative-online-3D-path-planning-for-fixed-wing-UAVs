function reducedpath = DouglasPeucker(path, epsilon)
% 道格拉斯-普克算法实现
% 输入:
% path - 原始曲线的坐标点, size为 Nx2, 每行是一个 (x, y) 坐标
% epsilon - 允许的最大误差 (阈值)


% 如果点数小于等于2, 直接返回
if size(path, 1) <= 2
    reducedpath = path;
    return;
end

% 找到离起点和终点的直线最远的点及其距离
start_point = path(1, :);
end_point = path(end, :);

% 初始化
max_distance = 0;
index = 0;

if size(path, 2) == 2
    distfun = @perpendicularDistance2D;
elseif size(path, 2) == 3
    distfun = @perpendicularDistance3D;
end

% 计算每个点到起点和终点连线的垂直距离
for i = 2 : size(path, 1) - 1
    distance = distfun(path(i, :), start_point, end_point);
    if distance > max_distance
        max_distance = distance;
        index = i;
    end
end

% 如果最大距离大于阈值 epsilon, 则递归分割曲线
if max_distance > epsilon
    % 递归调用算法，分为两部分
    left_path = DouglasPeucker(path(1 : index, :), epsilon);
    right_path = DouglasPeucker(path(index : end, :), epsilon);

    % 合并两个部分 (注意：合并时去掉右侧部分的第一个点，因为它是重复的)
    reducedpath = [left_path; right_path(2 : end, :)];
else
    % 如果所有点的距离都在阈值范围内，只保留起点和终点
    reducedpath = [start_point; end_point];
end
end

function d = perpendicularDistance3D(point, line_start, line_end)
% 计算点到三维空间中线段的垂直距离
% 输入: point为待测点, line_start和line_end为线段的起点和终点
% 输出: d为点到线段的垂直距离

% 如果起点和终点相同，返回点到该点的欧几里得距离
if isequal(line_start, line_end)
    d = norm(point - line_start);
    return;
end

% 计算点到线段的三维垂直距离
% 使用向量的叉积公式求解：
% d = |(P - A) x (B - A)| / |B - A|
% 其中 A 为线段起点，B 为线段终点，P 为点

vector_AB = line_end - line_start; % 向量 AB
vector_AP = point - line_start; % 向量 AP

% 计算向量 AB 和 AP 的叉积的模
cross_product = norm(cross(vector_AP, vector_AB));

% 计算向量 AB 的模
length_AB = norm(vector_AB);

% 点到线段的距离
d = cross_product / length_AB;
end



function dist = perpendicularDistance2D(point, lineStart, lineEnd)
% 计算点到二维空间中线段的垂直距离

x1 = lineStart(1); y1 = lineStart(2);
x2 = lineEnd(1); y2 = lineEnd(2);
x0 = point(1); y0 = point(2);

% 线段的向量方向
lineVec = [x2 - x1, y2 - y1];

% 点到线段的向量
pointVec = [x0 - x1, y0 - y1];

% 叉积计算垂直距离
crossProd = abs(lineVec(1) * pointVec(2) - lineVec(2) * pointVec(1));

% 线段的长度
lineLength = sqrt(lineVec(1) ^ 2 + lineVec(2) ^ 2);

% 垂直距离
dist = crossProd / lineLength;

end
