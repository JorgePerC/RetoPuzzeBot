function [z_p] = modeloSimplificado_accZ(psi)
    global plane envirmnt
        % gama, psi, vel, mass
    % envirmnt
        % g, p, S, C_d, v_fluid
    z_p = plane[3]*cos( plane[1] )*cos( plane[2] );

end
       