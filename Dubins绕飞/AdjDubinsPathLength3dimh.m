function [path, info] = AdjDubinsPathLength3dimh(startPose, goalPose, speed, dynamicCons, deltaLen, dir)

[~, length1, ~] = Get3dimDubinsPath(startPose, goalPose, speed, dynamicCons);
deltah = abs(goalPose(3) - startPose(3));
deltaL = sqrt((length1 + deltaLen) ^ 2 - (goalPose(3) - startPose(3)) ^ 2) - sqrt(length1 ^ 2 - (goalPose(3) - startPose(3)) ^ 2);
minR = speed ^ 2 / (9.8 * tan(dynamicCons.rollmax));
% k = mod(length1 / length, 1) / 2;
% k = 1 + k;
k = 1 + deltah / length1 / 70; % 经验修正系数
deltaL = deltaL * k;

[path2dim, path2dimLength, info] = AdjDubinsPathLength(startPose(1 : 2), startPose(4), goalPose(1 : 2), goalPose(4), minR, deltaL, dir);

dist2dim = vecnorm(diff(path2dim(:, 1 : 2)), 2, 2); % 相邻点之间的二维路径距离
cumdist = cumsum(dist2dim);
cumdist = [0; cumdist]; % 二维路径累加和

deltaZ = goalPose(3) - startPose(3); % 高度变化量
vz0 = speed * tan(startPose(5));

%tf = norm(goalPose(1 : 3) - startPose(1 : 3)) / speed * (1 + abs(pitchf - startPose(5)) / 2); % 乘以放大系数保证安全
tf = hypot(path2dimLength, goalPose(3) - startPose(3)) / speed;
tf = tf * 0.7;
t = tf * cumdist / path2dimLength;

vzf = speed * sin(goalPose(5));
deltaV = vzf - vz0;
x = [tf ^ 3 / 6, -tf ^ 2 / 2; tf ^ 2 / 2, -tf] \ [deltaZ - vz0 * tf; deltaV];
alpha = x(1);
beta = x(2);

z = alpha * t .^ 3 / 6 - beta * t .^ 2 / 2 + vz0 * t + startPose(3);
path = [path2dim(:, 1 : 2) double(z)]; % 实际存在误差

end
