clear all
close all
clc

theta = -15*pi/180;  % pitch (angulo de la tierra al vehiculo)
delta = -5*pi/180;   % attack angle (angulo del vehiculo al viento)

gamma = theta - delta; % Angulo entre el plano X-Y (la tierra) y el viento

psi = pi/4;

V = 10;

t = 0;
Tf = 10;
dt = 0.1;

x = 0;
y = 0;
z = 0;
while t < Tf

    psi = psi + 0.1*dt;
    xp = V*cos(psi)*cos(gamma);
    yp = V*cos(gamma)*sin(psi);
    zp = -V*sin(gamma);

    x = x + xp*dt;
    y = y + yp*dt;
    z = z + zp*dt;

    figure(1)
    scatter3(x,y,z);
    hold on
    axis([-100 100,-100 100,-100 100]);

    t = t + dt;
end
