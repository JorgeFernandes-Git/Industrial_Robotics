function T = Link_Trans(n,q,c,La,Lc,Le,Lf,Lg,Lh)

% This function returns the transformation matrix of the link
% n - link number
% q - angle of rotation of the link
% c - link position
% La,Lc,Le,Lf,Lg.Lh - robot dimensions

d = [La 0 0 Lf Lg Lh];
l = [0 Lc Le 0 0 0];
alph = [pi/2 0 0 pi/2 -pi/2 0];

T_l = eye(4);
T_l(1,4) = l(1,n);
T_d = eye(4);
T_d(3,4) = d(1,n);

a = q(n,c);
rzt = [cos(a) -sin(a) 0 0
       sin(a) cos(a)  0 0
       0        0     1 0
       0        0     0 1];

a = alph(1,n);
rxa = [1    0      0    0
       0 cos(a) -sin(a) 0
       0 sin(a) cos(a)  0
       0    0      0    1];

T = T_d*rzt*T_l*rxa;

