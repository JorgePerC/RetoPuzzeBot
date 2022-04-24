close all;
clc;

global robotConts;
            % x, y, lr, lf, 𝜃, 𝛿
robotConts = [0, 0,  2,  2, 0, 0];

bMap = readtable('ourMap.txt');

BinaryMap = table2array(bMap);

if exist('BinaryMap', 'var') == 0 
    BinaryMap = makemap(1000);
    writematrix(BinaryMap, 'ourMap.txt')
end 

figure() 
map = binaryOccupancyMap(rot90(transpose(BinaryMap)), 1000/50);

show(map)
nodes = 300;

tic 
planner = mobileRobotPRM(map, nodes);
toc 

figure()
show(planner)
startLocation = [5 30];
endLocation = [12 0];

tic
path = findpath(planner, startLocation, endLocation)
toc

figure()
show(planner) 

d_t = 0.1;

% Set the robot initial position and angle:
robotConts(1) = 0;
robotConts(2) = 35;
robotConts(5) = -pi/2;
L = robotConts(3) + robotConts(4);

kturn = 2;
klinear = .5;

for pos = 1: numel(path(:,1))

desiredPos = path(pos,:) %[10, 10]

for t = 0: d_t: 30
    %% Robot control 
    
    x_e = desiredPos(1, 1) - robotConts(1);
    y_e = desiredPos(1, 2) - robotConts(2);
    
    theta_e = atan2(y_e, x_e) - robotConts(5); 
    
    dist_e = hypot(x_e, y_e);

    v = dist_e * klinear;
    w = atan(2*L*sin(theta_e)/dist_e)*kturn;
    %w = theta_e * kturn*d_t;

    if abs(x_e) <= 0.5 && abs(y_e) <= 0.5 
        break
    end
    % limit v values
    if v < -1 
        v = -1;
    elseif v > 1 
        v = 1;
    end

    %% Update robot positon
    [x_dot, y_dot, theta_dot, steer_dot] = centerPoint(v, w);

    robotConts(1) = robotConts(1) + x_dot * d_t;
    robotConts(2) = robotConts(2) + y_dot * d_t;
    robotConts(5) = robotConts(5) + theta_dot * d_t;
        % Narrow down theta angles:
    if robotConts(5) > pi
        robotConts(5) = robotConts(5) - 2*pi;
    elseif robotConts(5) < -pi
        robotConts(5) = robotConts(5) + 2*pi;
    end
    robotConts(6) = robotConts(6) + steer_dot * d_t;
    % Limit the steering angle, since it cannot be more than
    % +- 30 degrees
    % http://street.umn.edu/VehControl/javahelp/HTML/Definition_of_Vehicle_Heading_and_Steeing_Angle.htm
    if robotConts(6) > pi/6
        robotConts(6) = pi/6;
    elseif robotConts(6) < -pi/6
        robotConts(6) = -pi/6;
    end

    % Graph:
    figure(5)
    scatter(robotConts(1), robotConts(2))
    hold on
    %drawnow
end

end