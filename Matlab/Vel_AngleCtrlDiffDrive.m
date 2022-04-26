function [x_d, y_d] = Vel_AngleCtrlDiffDrive(vel, angle)
    global robot;
    % L = robot(1)
    % R = robot(2)
    v_r = (2*vel + angle*robot(1))/(2*robot(2));
    v_l = (2*vel - angle*robot(1))/(2*robot(2));

    [x_d, y_d] = wheelVelCtrlDiffDrive(v_r, v_l);
end