function [P] = GenPoly(x0,xf,v0,vf,a0,af,Tf)

M = [0      0       0       0       0       1;
    Tf^5    Tf^4    Tf^3    Tf^2    Tf      1;
    0       0       0       0       1       0;
    5*Tf^4  4*Tf^3  3*Tf^2  2*Tf    1       0;
    0       0       0       2       0       0;
    20*Tf^3 12*Tf^2 6*Tf    2       0       0];

X = [x0;xf;v0;vf;a0;af];

P = pinv(M)*X;

end