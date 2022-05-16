function [v_p] = modeloSimplificado_accPlane(Thrust, g)
    global plane envirmnt
        % gama, psi, vel, mass
    % envirmnt
        % g, p, S, C_d, v_fluid
    % La velocidad del drag se mide respecto a la del viento
    d = 0.5* envirmnt[2]*(plane[3] - envirmnt[5])^2*envirmnt[3]*envirmnt[4];

    v_p = 1/plane[4]*(Thrust - d) - g*sin(plane[1]) 

end