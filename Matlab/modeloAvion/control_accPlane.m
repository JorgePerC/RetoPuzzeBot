function [th_out] = control_accPlane(v_ref)
    global plane envirmnt
        % gama, psi, vel, mass
    %envirmnt
        % g, p, S, C_d, v_fluid
    kp = 1;
    
    d = 0.5* envirmnt[2]*(plane[3] - envirmnt[5])^2*envirmnt[3]*envirmnt[4];

    th_out = -plane[4]*kp (plane[3] - v_ref) + D + envirmnt[1]*sin(plane[1]);

end
       