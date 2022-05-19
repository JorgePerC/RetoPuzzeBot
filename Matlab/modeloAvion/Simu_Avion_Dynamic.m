clear all
close all
clc

global plane envirmnt

    % gama, psi, vel, mass, theta, x, y, z
plane = [ 0,  0,   0,    1,     0, 0, 0, 0];

            %  g, p, S, C_d, v_fluid
envirmnt = [9.81, 1, 1,   1,       1];


% 
% plane(5) = -15*pi/180;  % pitch (angulo de la tierra al vehiculo)
% delta = -5*pi/180;   % attack angle (angulo del vehiculo al viento)
% 
% plane(2) = pi/4; % psi
% 
% plane(1) = plane(5) - delta; % Angulo entre el plano X-Y (la tierra) y el viento


t = 0;
Tf = 10;
dt = 0.1;

xp = 0;
yp = 0;
zp = 0;

kp = 0.5;

% Velocidad deseada
ref = 10;

while t < Tf
    % psi
    %plane(2) = plane(2) + 0.1*dt;
    

    D = 0.1*plane(3)^2;
    % Control
                        % z                     g*sin(gama)*mass
    thrust = ( kp*(ref - plane(8))  + envirmnt(1)*sin(plane(1)) )   *plane(4) + D;


    % plane vel
    plane(3) = modeloSimplificado_accPlane(thrust)


    xpp = modeloSimplificado_accX(plane(2));
    ypp = modeloSimplificado_accY(plane(2));
    zpp = modeloSimplificado_accZ(plane(1));

    xp = xp + xpp*dt;
    yp = yp + ypp*dt;
    zp = zp + zpp*dt;

    % x
    plane(6) = plane(6) + xp*dt;
    % y
    plane(7) = plane(7) + yp*dt;
    % z
    plane(8) = plane(8) + zp*dt;



    figure(1)
    scatter3(plane(6), plane(7), plane(8));
    hold on
    axis([-100 100,-100 100,-100 100]);

    t = t + dt;
end
