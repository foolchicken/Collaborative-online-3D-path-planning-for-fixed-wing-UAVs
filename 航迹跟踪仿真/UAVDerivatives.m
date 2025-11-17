function [dydt, status] = UAVDerivatives(y, wpFollowerObj, LookAheadDist, model, e, PHeadingAngle, airSpeed, rollAngleLimit)
%计算无人机的状态导数，便于后续积分函数
%输出状态导数和当前过点状态，若经过全部航路点status=1

[lookAheadPoint, desiredHeading,~,~,~,status] = wpFollowerObj([y(1); y(2); y(3); y(5)], LookAheadDist);
% 根据L1制导率计算正在跟踪的航路点和期望航向角
% lookAheadPoint和y(1:2)之间的距离严格等于LookAheadDist
% lookAheadPoint在matlab的内部计算较为麻烦
% 首先根据当前位置和判过点距离计算需要跟踪的两个点
% 然后根据当前位置和需要跟踪的两个点使用二次函数拟合曲线
% 最后在从二次曲线上选取一个到当前位置等于LookAheadDist的点

%NEH to NED frame conversion
desiredHeight = -lookAheadPoint(3); % 以期望航路点的高度作为跟踪高度（NED坐标系，z指向下）
RollAngle = UAVHeadingControl(y, desiredHeading, e, PHeadingAngle, rollAngleLimit); % 根据制导模型、环境计算期望滚转角

% 控制量为滚转角、高度和空速
u = control(model);
u.RollAngle = RollAngle;
u.Height = desiredHeight;
u.AirSpeed = airSpeed;

% convert to NED frame
yNED = y;
yNED(3) = -y(3);
dydtNED = derivative(model, yNED, u, e); % 根据模型、控制量输入和环境计算导数

%convert from NED to NEU frame back
dydt = dydtNED;
dydt(3) = -dydtNED(3);

end
