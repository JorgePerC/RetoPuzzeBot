function [x_d, y_d] = wheelVelCtrlDiffDrive(rw_dv, lw_dv)
    global robot d_t;
    % L = robot(1)
    % R = robot(2)
    % w = robot(5)

    V = robot(2)*(rw_dv + lw_dv)/2;
    w = robot(2)*(rw_dv - lw_dv)/robot(1);
    
    x_d = V* cos(robot(5));
    y_d = V* sin(robot(5));
    w_d = w;

    % Update robot pos:
    robot(3) = robot(3) + x_d*d_t; % TODO: reference d_t
    robot(4) = robot(4) + y_d*d_t;
    % Update robot angle:
    robot(5) = robot(5) + w_d*d_t;

end