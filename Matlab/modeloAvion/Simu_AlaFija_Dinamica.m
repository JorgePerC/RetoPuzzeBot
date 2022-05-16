clear all
close all
clc

theta = -15*pi/180;  % pitch (angulo de la tierra al vehiculo)
delta = -5*pi/180;   % attack angle (angulo del vehiculo al viento)

gamma = theta - delta; % Angulo entre el plano X-Y (la tierra) y el viento

psi = pi/4;

V = 10;
Fth = 2;
m = 1;

t = 0;
Tf = 10;
dt = 0.1;

x = 0;
y = 0;
z = 0;

xp = 0;
yp = 0;
zp = 0;
while t < Tf

    L = 0.1*V^2;
    D = 0.1*V^2;

    psi = psi + 0.1*dt;


    xpp = L*cos(psi)*sin(gamma) - (Fth-D)*cos(psi)*cos(gamma);
    ypp = L*sin(psi)*sin(gamma) - (Fth-D)*sin(psi)*cos(gamma);
    zpp = L*cos(gamma) + (Fth-D)*sin(gamma) - m*9.81;

    xp = xp + xpp*dt;
    yp = yp + ypp*dt;
    zp = zp + zpp*dt;

    x = x + xp*dt;
    y = y + yp*dt;
    z = z + zp*dt;

    figure(1)
    scatter3(x,y,z);
    hold on
    axis([-100 100,-100 100,-100 100]);

    t = t + dt;
end
