function [ vu, omega, Controller, CTE, Data] = calculateControlOutput( robotPose, goalPose, prev_vu, prev_omega, parameters, Controller, CTE, Data)
%CALCULATECONTROLOUTPUT This function computes the motor velocities for a differential driven robot

% current robot position and orientation
x = robotPose(1);
y = robotPose(2);
theta = robotPose(3);

% goal position and orientation
xg = goalPose(1);
yg = goalPose(2);
thetag = goalPose(3);

% compute control quantities
rho = sqrt((xg-x)^2+(yg-y)^2);  % pythagoras theorem, sqrt(dx^2 + dy^2)
lambda = atan2(yg-y, xg-x);     % angle of the vector pointing from the robot to the goal in the inertial frame
alpha = lambda - theta;         % angle of the vector pointing from the robot to the goal in the robot frame
alpha = normalizeAngle(alpha);
beta = abs(normalizeAngle(theta-thetag))-alpha;

% the following paramerters should be used:
% Task 2:
% parameters.Kalpha, parameters.Kbeta, parameters.Krho: controller tuning parameters
% Task 3:
% parameters.backwardAllowed: This boolean variable should switch the between the two controllers
% parameters.useConstantSpeed: Turn on constant speed option
% parameters.constantSpeed: The speed used when constant speed option is on

% Controller Parameters
dt = Controller.timestamp;

% Parameters Calculation
curvature = (2*alpha)/(rho^2);
cte = rho*sin(alpha);

[CTEoutput, CTE] = PIDController.updateCTE(CTE, cte, dt);

vu = parameters.Krho*(rho+CTEoutput); % [m/s]
% vu = parameters.Krho*(rho); % [m/s]
omega = parameters.Kalpha*(alpha) + parameters.Kbeta*(beta) + (2*vu)*CTEoutput/(CTE.ld^2); % [rad/s]

% Restriction
if (vu > 2)
    vu = prev_vu;
elseif (vu < 0)
    vu = 0;
end
vu = (vu+prev_vu)/2;
omega = (omega+prev_omega)/2;

% Data Recording
Data.curvature = [Data.curvature; curvature];
Data.CrossTrackError = [Data.CrossTrackError; cte];
end
