function rollAngle = UAVHeadingControl(flightState, desiredHeading, environment, PHeadingAngle, rollAngleLimit)
%根据期望航向角计算滚转角

Va = flightState(4);
headingAngle = flightState(5);
flightPathAngle = flightState(6);

% 根据航迹偏角计算航向角 参考书第23页
% 无风环境下YawAngle=headingAngle
YawAngle = headingAngle - asin((1 / Va) * [environment.WindNorth, environment.WindEast] * [-sin(headingAngle); cos(headingAngle)]);
% 根据空速 航迹角计算地速 参考书第23页
b = [cos(headingAngle) * cos(flightPathAngle),...
    sin(headingAngle) * cos(flightPathAngle),...
    -sin(flightPathAngle)] * ...
    [environment.WindNorth;
    environment.WindEast;
    environment.WindDown];
c = norm([environment.WindNorth, environment.WindEast, environment.WindDown])^2 - Va ^ 2;
Vg = b + sqrt(b ^ 2 - c); % b前面的正负号有疑问
% 根据期望航迹偏角 实际航迹偏角 航向角计算期望滚转角
% 协调转弯，参考书第65页
dheadingAngle = PHeadingAngle * angdiff(headingAngle, desiredHeading); % 期望航向角速度
rollAngle = atan2(dheadingAngle * Vg, environment.Gravity * cos(headingAngle - YawAngle)); % BTT转弯计算期望滚转角

if(rollAngle > abs(rollAngleLimit))
    rollAngle = abs(rollAngleLimit);
end
if(rollAngle < -abs(rollAngleLimit))
    rollAngle = -abs(rollAngleLimit);
end

end
