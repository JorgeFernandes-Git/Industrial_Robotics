function Q = invkinUR10(x,y,z,La,Lc,Le,Lf,Lg,Lh)

% returns the angles for the joints based on the inverse kinematics of the robot
% x,y,z - coordinates of the end-effector
% La,Lc,Le,Lf,Lg.Lh - robot dimensions

Q = zeros(6,8);

% https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf
% https://github.com/mc-capolei/python-Universal-robot-kinematics
% https://www.slideshare.net/RyanKeating13/ur5-ik

% x = [nx ox ax px]
% y = [ny oy ay py]
% z = [nz oz az pz]

% La = d1 -90
% Lc = a2 0
% Le = a3 0
% Lf = d4 0
% lg = d5 0
% Lh = d6 180

dp = [x;y;z;0 0 0 1];
P_05 = dp * [0 0 -Lh 1]' - [0 0 0 1]';

% **** theta1 ****
psi = atan2(P_05(2,1),P_05(1,1));
phi = acos(Lf / sqrt(P_05(1,1)^2 + P_05(2,1)^2));

q1A = pi/2 + psi + phi;
q1B = pi/2 + psi - phi;
q1 = [q1A q1B];

Q(1,1:4) = q1A;
Q(1,5:8) = q1B;

Q = real(Q);

% **** theta5 ****
pxS1 = dp(1,4)*sin(q1);
pyC1 = dp(2,4)*cos(q1);
q5A = acos((pxS1 - pyC1 - Lf)/Lh);
q5B = - acos((pxS1 - pyC1 - Lf)/Lh);
q5 = [q5A q5B];

Q(5,1:2) = q5A(1);
Q(5,3:4) = q5B(1);
Q(5,5:6) = q5A(2);
Q(5,7:8) = q5B(2);

Q = real(Q);

% **** theta6 ****
% theta6 is not well-defined when sin(theta5) = 0
yxS1 = [dp(1,2) * sin(q1) dp(1,2) * sin(q1)];
yyC1 = [dp(2,2) * cos(q1) dp(2,2) * cos(q1)];
xxS1 = [dp(1,1) * sin(q1) dp(1,1) * sin(q1)];
xyC1 = [dp(2,1) * cos(q1) dp(2,1) * cos(q1)];

q6 = atan2((-yxS1+yyC1)./sin(q5), (-(-xxS1+xyC1)./sin(q5)));
Q(6, 1:2) = q6(1);
Q(6, 3:4) = q6(3);
Q(6, 5:6) = q6(2);
Q(6, 7:8) = q6(4);

Q = real(Q);

% **** theta3 ****
cl = [1 3 5 7];
for i=1:width(cl)
    c = cl(i);
    T_10 = inv(Link_Trans(1,Q,c,La,Lc,Le,Lf,Lg,Lh));
    T_65 = Link_Trans(6,Q,c,La,Lc,Le,Lf,Lg,Lh);
    T_54 = Link_Trans(5,Q,c,La,Lc,Le,Lf,Lg,Lh);
    T_14 = (T_10 * dp) * inv(T_54*T_65);
    P_13 = T_14 * [0 -Lf 0 1]' - [0 0 0 1]';
    q3 = acos((norm(P_13,'fro')^2 - Lc^2 - Le^2)./(2*Lc*Le));
    Q(3,c) = q3;
    Q(3,c+1) = -q3;
end

Q = real(Q);

% **** theta2 and theta 4 ****
cl = [1 2 3 4 5 6 7 8];
for i = 1:width(cl)
    c = cl(i);
    T_10 = inv(Link_Trans(1,Q,c,La,Lc,Le,Lf,Lg,Lh));
    T_65 = Link_Trans(6,Q,c,La,Lc,Le,Lf,Lg,Lh);
    T_54 = Link_Trans(5,Q,c,La,Lc,Le,Lf,Lg,Lh);
    T_14 = (T_10 * dp) * inv(T_54*T_65);
    P_13 = T_14 * [0 -Lf 0 1]' - [0 0 0 1]';

    % **** theta2
    Q(2,c) = -atan2(P_13(2),-P_13(1))+asin(Le * sin(Q(3,c))./norm(P_13,'fro'));
    T_32 = inv(Link_Trans(3,Q,c,La,Lc,Le,Lf,Lg,Lh));
    T_21 = inv(Link_Trans(2,Q,c,La,Lc,Le,Lf,Lg,Lh));
    T_34 = T_32 * T_21 * T_14;
    Q(4,c) = atan2(T_34(2,1),T_34(1,1));
end

Q = real(Q);
Q = rad2deg(Q);