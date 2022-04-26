close all;
clear all;
clc;

global robot d_t;

L = 0.5;
R = 1;
initX = 5;
initY = 30;
intit_w = 3/2*pi;

% Constants for our diff drive 
robot = [ L, R, initX ,initY, intit_w];

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

i = 1;
d_t = 0.1;

robotPathX(1) = 0;
robotPathY(1) = 0;
suggTrayectoryX(1) = 0;
suggTrayectoryX(1) = 0;


% Control 
kt = 0.8;
ka = 1;

velX_last = 0;
velY_last = 0;
velR_last = 1;

timeP2P = 13;

for pos = 1: numel(path(:,1))
    %% Calculate the trayectory for each point in the path

    desiredPos = path(pos,:); %[10, 10]
    
    thetaV = atan2(desiredPos(1, 2) - robot(4), desiredPos(1, 1) - robot(3));
    vxf = velR_last*cos(thetaV);
    vyf = velR_last*sin(thetaV);

                % TODO: Try how it behaves with lastPointX
    Px = GenPoly(robot(3), ... % Robot actual position in x
                desiredPos(1, 1), ... % Next mini-point x
                velX_last, ... % Robot actual velocity, to avoid constant stops | velX_last
                vxf, ...
                0,0, ... % At 1st run, from stop and then keep vel
                timeP2P); % YOLO

    Py = GenPoly(robot(4), ... % Robot actual position in y
                desiredPos(1, 2), ... % Next mini-point y
                velY_last, ... % Robot actual velocity, to avoid constant stops
                vyf, ...
                0,0, ... % At 1st run, from stop and then keep vel
                timeP2P); % YOLO
    
    %% Navigate through trayectory
    t = 0;

    while t <= timeP2P
        % i stands for iteration

        x_i = Px(1)*t^5 + Px(2)*t^4 + Px(3)*t^3 + Px(4)*t^2 + Px(5)*t + Px(6);
        % We believe these are useless
        vx_i = 5*Px(1)*t^4 + 4*Px(2)*t^3 + 3*Px(3)*t^2 + 2*Px(4)*t + Px(5);
        ax_i = 20*Px(1)*t^3 + 12*Px(2)*t^2 + 6*Px(3)*t + 2*Px(4);
    
        y_i = Py(1)*t^5 + Py(2)*t^4 + Py(3)*t^3 + Py(4)*t^2 + Py(5)*t + Py(6);
        % We believe these are useless
        vy_i = 5*Py(1)*t^4 + 4*Py(2)*t^3 + 3*Py(3)*t^2 + 2*Py(4)*t + Py(5);
        ay_i = 20*Py(1)*t^3 + 12*Py(2)*t^2 + 6*Py(3)*t + 2*Py(4);    
    
            %% Robot control 
            
        x_e = x_i - robot(3);
        y_e = y_i - robot(4);
        
        theta_e = atan2(y_e, x_e) - robot(5); 
        
        dist_e = hypot(x_e, y_e);

        v = kt*dist_e;
        w = ka*theta_e;
    
        if v > 1
            v = 1;
        end
        if w > pi/2
            w = pi/2;
        end
        if w < -pi/2
            w = -pi/2;
        end
        
        if abs(desiredPos(1, 1) -robot(3)) <= 0.5 && abs(desiredPos(1, 2) -robot(4)) <= 0.5 
            break
        end
        
            %% Update robot positon
            [x_dot, y_dot] = Vel_AngleCtrlDiffDrive(v,w);
        
            robotPathX(i) = robot(3);
            robotPathY(i) = robot(4);
            suggTrayectoryX(i)  = x_i;
            suggTrayectoryY(i)  = y_i;
            
            %drawnow
        %end
        t = t + d_t;
        i = i+1;
        % Update last velocity TODO: VERIFY
        velX_last = x_dot; %vy_i
        velY_last = y_dot; %vy_i | vyf
        velR_last = v;
    end
    
end


% Graph:
figure(5)
show(planner) 
hold on
scatter(robotPathX, robotPathY)

figure(6)
show(planner)
hold on
scatter(suggTrayectoryX, suggTrayectoryY)
