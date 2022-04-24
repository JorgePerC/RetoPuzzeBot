function [x_dot, y_dot, theta_dot, steer_dot] = centerPoint(v, w)

% Global robot variables:
global robotConts

% Robot variables:
theta = robotConts(5);
steer = robotConts(6);

L = robotConts(3) + robotConts(4);

splipAngle = atan(robotConts(3) * tan(steer) / L);

S = L / tan(steer);

R = S / cos(splipAngle);

x_dot = v * cos(splipAngle + theta);
y_dot = v* sin(splipAngle + theta);

theta_dot = v / R;
steer_dot = w;

end