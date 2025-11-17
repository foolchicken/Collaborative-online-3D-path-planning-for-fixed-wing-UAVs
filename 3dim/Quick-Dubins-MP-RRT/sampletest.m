close all
clear
clc

algoParam.start = [103.735289, 39.56, 1700, deg2rad(90), deg2rad(0)]; % 经纬高、航向角（和x轴夹角）、俯仰角
algoParam.goal = [103.665, 39.56, 2000, deg2rad(180), deg2rad(0)];
algoParam.mapRef.LongitudeLimits = [0 1]; 
algoParam.mapRef.LatitudeLimits = [0, 1];
algoParam.resolutionMP = 100;

figure
scatter3(0, 0, algoParam.start(3));
hold on
for i = 1 : 500
    sample = ChooseSample(algoParam, 3827.58405825896, 1.922735572511179e+03, 10);
    scatter3(sample(1), sample(2), sample(3))
end
hold off
grid on
xlabel('x')
ylabel('y')

p = [1;0;0];
yaw = pi;
pitch = pi/6;
Lyaw = [cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
Lpitch = [cos(pitch) 0 -sin(pitch);0 1 0;sin(pitch) 0 cos(pitch)];
pnew = Lyaw*Lpitch*p

