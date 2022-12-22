function rTh = Multi_Trans(q,La,Lc,Le,Lf,Lg,Lh)

% Returns the transformation matrix of multiple links
% q - angle of rotation of the link

A1=rotz(q(1))*trans(0,0,La)*rotx(90);
A2=rotz(q(2))*trans(Lc,0,0)*rotx(0);
A3=rotz(q(3))*trans(Le,0,0)*rotx(0);
A4=rotz(q(4))*trans(0,0,Lf)*rotx(90);
A5=rotz(q(5))*trans(0,0,Lg)*rotx(-90);
A6=rotz(q(6))*trans(0,0,Lh)*rotx(0);

rTh = A1*A2*A3*A4*A5*A6;