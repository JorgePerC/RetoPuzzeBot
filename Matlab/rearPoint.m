% Bicicle model

% The side sleep angle depends on the reference point in our model
% That's why we'll create a different function for the rearWheel, 
% frontWheel and, bicycleCenter

% Remember that is is a discrete time model, and what we'll do 

function [x_dot, y_dot, theta_dot, steer_dot] = rearPoint(v, w)

   global robotConts;
    
    % Robot variables:
        % TODO: Pass inital 
    theta = robotConts(5);
    steer = robotConts(6);
    
    L = robotConts(3) + robotConts(4);

    % This R is different from the center of the model, so...
    % The formula changes: https://www.youtube.com/watch?v=HqNdBiej23I
    R = L / tan(steer);

    % Normally would be defined as: v* cos(theta + slipAngle)
    % Tho, with the rearWheel as a reference point, slipAngle = 0
    x_dot = v* cos(theta);
    y_dot = v* sin(theta);
    
    % Rotation rate:
            % Given by the angular velocity compared to the ICR 
    theta_dot = v/R;
    steer_dot = w;

end




