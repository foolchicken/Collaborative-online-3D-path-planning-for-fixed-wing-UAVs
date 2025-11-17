function uavState = UAVPathFollowSimulate(waypoints, initialState, L1, TransitionRadius, dynamicCons)
% 无人机路径跟踪仿真，输入航路点、初始状态、L1距离、过点判断距离、动力学约束

% Define Model and gains for the fixed wing guidance model.
model = fixedwing;
model.Configuration.PDRoll = [20 4];
model.Configuration.PHeight = 0.07;
model.Configuration.PFlightPathAngle = 1;
model.Configuration.PAirSpeed = 0.39;
model.Configuration.FlightPathAngleLimits = [dynamicCons.pitchmin dynamicCons.pitchmax];

% Setup environment struct for fixed wing guidance model
e = environment(model);

% Setup model initial states
% The states are in the format given below:
% North, East, Height, AirSpeed, Heading Angle, FlightPathAngle, RollAngle, RollAngleRate
y0 = state(model);
y0(1 : 6) = initialState;
speed = initialState(4);
lenexp = sum(vecnorm(diff(waypoints),2,2)); % 期望飞行长度
texp = lenexp/speed; % 期望飞行时间

% Setup WayPoint Follower Object
wpFollowerObj = uavWaypointFollower('UAVType', 'fixed-wing', 'Waypoints', waypoints(:, 1 : 3), 'TransitionRadius', TransitionRadius, 'StartFrom', 'First');

% Set Heading Control Gain for Heading Controller.
PHeadingAngle = 2;

% Define Roll Angle Limit for UAV
UAVRollLimit = dynamicCons.rollmax;

% 使用龙格库塔求解，普通欧拉会发散，并且时间步长不能太大
uavState = y0';
iter = 1;
dt = 0.1;
time = 0;
lenpassed = 0; % 已经走过的路程
speedexp = speed; % 期望速度
while 1
    y = uavState(iter, :)';
    [k1, status] = UAVDerivatives(y, wpFollowerObj, L1, model, e, PHeadingAngle, speedexp, UAVRollLimit);
    k2 = UAVDerivatives((y + k1 * dt / 2), wpFollowerObj, L1, model, e, PHeadingAngle, speedexp, UAVRollLimit);
    k3 = UAVDerivatives((y + k2 * dt / 2), wpFollowerObj, L1, model, e, PHeadingAngle, speedexp, UAVRollLimit);
    k4 = UAVDerivatives((y + k3 * dt), wpFollowerObj, L1, model, e, PHeadingAngle, speedexp, UAVRollLimit);
    uavState(iter + 1, :) = uavState(iter, :) + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)';
    lenpassed = lenpassed + norm(uavState(iter + 1, :) - uavState(iter, :));
    time(iter + 1) = time(iter) + dt;
    speedexp = (lenexp - lenpassed)/(texp - time(iter+1));
    if status == 1
        A = uavState(end, 1 : 3) - waypoints(end, 1 : 3);
        B = waypoints(end, 1 : 3) - waypoints(end - 1, 1 : 3);
        theta = acos(dot(A, B) / (norm(A) * norm(B))); % 相平面法判最终过点
        if theta < pi / 2
            break
        end
    end
    iter = iter + 1;
end
uavState = [uavState, time'];

% simOut = ode45(@(t,y)UAVDerivatives(y,wpFollowerObj,L1,model,e,PHeadingAngle,speed,UAVRollLimit), linspace(0,20,100),y0);
% uavState = simOut.y';

end
