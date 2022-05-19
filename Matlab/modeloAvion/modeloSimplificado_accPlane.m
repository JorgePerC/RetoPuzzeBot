function [v_p] = modeloSimplificado_accPlane(Thrust)
    global plane envirmnt
        % gama, psi, vel, mass
    % envirmnt
        % g, p, S, C_d, v_fluid
    % La velocidad del drag se mide respecto a la del viento
    d = 0.5* envirmnt(2)*(plane(3) - envirmnt(5))^2*envirmnt(3)*envirmnt(4);

    % TODO: Remove this for a real calculation
            % vel²
    d = 0.1*plane(3)^2;
            % m                             g*sin(gama)
    v_p = 1/plane(4)*(Thrust - d) - envirmnt(1)*sin(plane(1));

end