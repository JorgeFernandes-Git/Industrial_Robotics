clc
clear
close all

addpath functions\

debug_mode = false; % for debug only
plot_joints = false; % generate plot of the joints
draw_end_path = true; % draw end-effector path

try
    tfid = fopen('tp2.txt');
    tdata = textscan(tfid,'%s=%s');
    fclose(tfid);
    if( numel(tdata{1}) ~= numel(tdata{2}))
        disp('Error reading file. Missing = !')
        clear tdata tfid
    else
        ndata={ tdata{1} repmat('=', size(tdata{1})) tdata{2}};
        sdata=strcat(ndata{1},ndata{2},ndata{3});
        for i=1:numel(sdata)
            try
                eval(sdata{i});
            catch
                sprintf('Bad format in line %d of data file!',i)
            end
        end
        clear i tfid ndata tdata sdata
    end
catch
    disp('Cannot open file.')
end

% dimensions
if ~exist('LA','var')
    LA = 181;
    LB = 176;
    LC = -613;
    LD = 137;
    LE = -572;
    LF = 135;
    LG = 120;
    LH = 332;
    DA = 550;
    DB = 340;
    HT = 560;
    DT = 410;
    DL = 1540;
    DT1 = 440;
    DT2 = 370;
end

if LC > 0
    LC = -LC;
end
if LE > 0
    LE = -LE;
end

fh = figure();
if debug_mode == 0
    fh.WindowState = 'maximized';
end
hold on
axis off
grid on

% view(90,0) % y,z
% view(90,90) % x,y
% view(0,90) % x,y

view(120,20)
axis_dim = 3000;
axis image
axis([-50 axis_dim -50 axis_dim 0 2200])
% axis square
xlabel('x')
ylabel('y')
zlabel('z')

% gripper
fingers_l = 50;
LH = LH-fingers_l;

% max range
max_range = abs(LC)+abs(LE)+LH;

%%%%%%%%%% C1 %%%%%%%%%%
C1_org = [0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 50
    0 100 50
    100 100 50
    100 0 50
    50 26 50
    74 50 50
    50 74 50
    26 50 50
    50 26 200
    74 50 200
    50 74 200
    26 50 200];

random_pose = rand(1) * 500;
random_pose_draw_C1 = random_pose;
h_C1_org = [C1_org'; ones(1,size(C1_org,1))];
h_C1_org = trans(905+125+DL-160,1250+DT2-125+125+DT1+random_pose,HT)*h_C1_org;
C1_org = h_C1_org(1:3,:)';

C1_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
    9 10 11 12
    13 14 15 16
    9 10 14 13
    10 11 15 14
    11 12 16 15
    12 9 13 16];

C1 = patch('Vertices',C1_org,'Faces',C1_Faces,'FaceColor',"#535454");
%%%%%%%%%% C1 %%%%%%%%%%

%%%%%%%%%% C2 %%%%%%%%%%
C2_org = [0 0 0
    0 35 0
    11 35 0
    35 11 0
    35 0 0
    0 0 50
    0 35 50
    11 35 50
    35 11 50
    35 0 50

    0 35 0
    11 35 0
    35 59 0
    35 70 0
    0 70 0
    0 35 50
    11 35 50
    35 59 50
    35 70 50
    0 70 50

    35 0 0
    35 11 0
    59 35 0
    70 35 0
    70 0 0
    35 0 50
    35 11 50
    59 35 50
    70 35 50
    70 0 50

    35 70 0
    35 59 0
    59 35 0
    70 35 0
    70 70 0
    35 70 50
    35 59 50
    59 35 50
    70 35 50
    70 70 50
    ];

h_C2_org = [C2_org'; ones(1,size(C2_org,1))];
h_C2_org_ori = h_C2_org;
h_C2_org_ori = rotx(90)*h_C2_org_ori;
h_C2_org_ori = trans(-35,25,-13)*h_C2_org_ori;
random_pose = rand(1)*500;
random_pose_draw_C2 = random_pose;
h_C2_org = trans(1300+random_pose+(DT-410),200,HT)*h_C2_org;
C2_org = h_C2_org(1:3,:)';

C2_Faces = [1 2 3 4 5
    6 7 8 9 10
    1 2 2 7 6
    2 3 3 8 7
    3 4 4 9 8
    4 5 5 10 9
    5 1 1 6 10

    11 12 13 14 15
    16 17 18 19 20
    11 12 12 17 16
    12 13 13 18 17
    13 14 14 19 18
    14 15 15 20 19
    15 11 11 16 20

    21    22    23    24    25
    26    27    28    29    30
    21    22    22    27    26
    22    23    23    28    27
    23    24    24    29    28
    24    25    25    30    29
    25    21    21    26    30

    31    32    33    34    35
    36    37    38    39    40
    31    32    32    37    36
    32    33    33    38    37
    33    34    34    39    38
    34    35    35    40    39
    35    31    31    36    40
    ];

C2 = patch('Vertices',C2_org,'Faces',C2_Faces,'FaceColor',"#535454");
%%%%%%%%%% C2 %%%%%%%%%%

%%%%%%%%%% C3 %%%%%%%%%%
t = 0.05:0.5:2*pi;
x1 = 30*cos(t);
y1 = 30*sin(t);
C3_1 = [x1' y1' zeros(size(x1))'];
C3_2 = C3_1;
C3_2(:,3) = 50;
C3_org = [C3_1;C3_2

-24 0 0
0 24 0
24 0 0
0 -24 0
-24 0 50
0 24 50
24 0 50
0 -24 50];

h_C3_org = [C3_org'; ones(1,size(C3_org,1))];
h_C3_org_ori = h_C3_org;
h_C3_org_ori = rotx(90)*h_C3_org_ori;
h_C3_org_ori = trans(0,25,18)*h_C3_org_ori;
random_pose = rand(1) * 300;
random_pose_draw_C3 = random_pose;
h_C3_org = trans(300+random_pose,230,HT)*h_C3_org;
C3_org = h_C3_org(1:3,:)';

C3_Faces = [1 2 3 4 5 6 7 8 9 10 11 12 13
    14 15 16 17 18 19 20 21 22 23 24 25 26
    1 2 15 15 15 15 15 15 15 15 15 15 14
    2 3 16 16 16 16 16 16 16 16 16 16 15
    3 4 17 17 17 17 17 17 17 17 17 17 16
    4 5 18 18 18 18 18 18 18 18 18 18 17
    5 6 19 19 19 19 19 19 19 19 19 19 18
    6 7 20 20 20 20 20 20 20 20 20 20 19
    7 8 21 21 21 21 21 21 21 21 21 21 20
    8 9 22 22 22 22 22 22 22 22 22 22 21
    9 10 23 23 23 23 23 23 23 23 23 23 22
    10 11 24 24 24 24 24 24 24 24 24 24 23
    11 12 25 25 25 25 25 25 25 25 25 25 24
    12 13 26 26 26 26 26 26 26 26 26 26 25
    13 1 1 1 1 1 1 1 1 1 1 14 26

    27 28 28 28 28 28 28 28 28 28 28 29 30
    31 32 33 33 33 33 33 33 33 33 33 33 34
    27 31 32 32 32 32 32 32 32 32 32 32 28
    28 32 33 33 33 33 33 33 33 33 33 33 29
    29 33 34 34 34 34 34 34 34 34 34 34 30
    30 34 27 27 27 27 27 27 27 27 27 27 27
    ];

C3 = patch('Vertices',C3_org,'Faces',C3_Faces,'FaceColor',"#535454");
%%%%%%%%%% C3 %%%%%%%%%%

%%%%%%%%%% T1 %%%%%%%%%%
T1_org = [0 0 HT
    0 2560 HT
    625 2560 HT
    625 0 HT
    0 0 HT-60
    0 2560 HT-60
    625 2560 HT-60
    625 0 HT-60

    0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 HT-60
    0 100 HT-60
    100 100 HT-60
    100 0 HT-60

    625 0 0
    525 0 0
    525 100 0
    625 100 0
    625 0 HT-60
    525 0 HT-60
    525 100 HT-60
    625 100 HT-60

    0 2560 0
    100 2560 0
    100 2460 0
    0 2460 0
    0 2560 HT-60
    100 2560 HT-60
    100 2460 HT-60
    0 2460 HT-60

    625 2560 0
    625 2460 0
    525 2460 0
    525 2560 0
    625 2560 HT-60
    625 2460 HT-60
    525 2460 HT-60
    525 2560 HT-60
    ];

T1_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8

    9    10    11    12
    13    14    15    16
    9    10    14    13
    10    11    15    14
    11    12    16    15
    12     9    13    16

    17    18    19    20
    21    22    23    24
    17    18    22    21
    18    19    23    22
    19    20    24    23
    20    17    21    24

    25    26    27    28
    29    30    31    32
    25    26    30    29
    26    27    31    30
    27    28    32    31
    28    25    29    32

    33    34    35    36
    37    38    39    40
    33    34    38    37
    34    35    39    38
    35    36    40    39
    36    33    37    40
    ];

T1_org = [T1_org'; ones(1,size(T1_org,1))];
T1_org = rotz(90)*T1_org;
T1_org = trans(905+125+DL,1250+DT2-125+125+DT1,0)*T1_org;
T1_org = T1_org(1:3,:)';

T1 = patch('Vertices',T1_org,'Faces',T1_Faces,'FaceColor',"#02613e");
%%%%%%%%%% T1 %%%%%%%%%%

%%%%%%%%%% T2 %%%%%%%%%%
T2_org = [0 0 HT
    0 1250 HT
    625 1250 HT
    625 0 HT
    0 0 HT-60
    0 1250 HT-60
    625 1250 HT-60
    625 0 HT-60

    0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 HT-60
    0 100 HT-60
    100 100 HT-60
    100 0 HT-60

    625 0 0
    525 0 0
    525 100 0
    625 100 0
    625 0 HT-60
    525 0 HT-60
    525 100 HT-60
    625 100 HT-60

    0 1250 0
    100 1250 0
    100 1150 0
    0 1150 0
    0 1250 HT-60
    100 1250 HT-60
    100 1150 HT-60
    0 1150 HT-60

    625 1250 0
    625 1150 0
    525 1150 0
    525 1250 0
    625 1250 HT-60
    625 1150 HT-60
    525 1150 HT-60
    525 1250 HT-60
    ];

T2_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8

    9    10    11    12
    13    14    15    16
    9    10    14    13
    10    11    15    14
    11    12    16    15
    12     9    13    16

    17    18    19    20
    21    22    23    24
    17    18    22    21
    18    19    23    22
    19    20    24    23
    20    17    21    24

    25    26    27    28
    29    30    31    32
    25    26    30    29
    26    27    31    30
    27    28    32    31
    28    25    29    32

    33    34    35    36
    37    38    39    40
    33    34    38    37
    34    35    39    38
    35    36    40    39
    36    33    37    40
    ];

T2_org = [T2_org'; ones(1,size(T2_org,1))];
T2_org = trans(200,0,0)*T2_org;
T2_org = T2_org(1:3,:)';

T2 = patch('Vertices',T2_org,'Faces',T2_Faces,'FaceColor',"#02613e");
%%%%%%%%%% T2 %%%%%%%%%%

%%%%%%%%%% T3 %%%%%%%%%%
T3_org = [0 0 HT
    0 1250 HT
    625 1250 HT
    625 0 HT
    0 0 HT-60
    0 1250 HT-60
    625 1250 HT-60
    625 0 HT-60

    0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 HT-60
    0 100 HT-60
    100 100 HT-60
    100 0 HT-60

    625 0 0
    525 0 0
    525 100 0
    625 100 0
    625 0 HT-60
    525 0 HT-60
    525 100 HT-60
    625 100 HT-60

    0 1250 0
    100 1250 0
    100 1150 0
    0 1150 0
    0 1250 HT-60
    100 1250 HT-60
    100 1150 HT-60
    0 1150 HT-60

    625 1250 0
    625 1150 0
    525 1150 0
    525 1250 0
    625 1250 HT-60
    625 1150 HT-60
    525 1150 HT-60
    525 1250 HT-60
    ];

T3_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8

    9    10    11    12
    13    14    15    16
    9    10    14    13
    10    11    15    14
    11    12    16    15
    12     9    13    16

    17    18    19    20
    21    22    23    24
    17    18    22    21
    18    19    23    22
    19    20    24    23
    20    17    21    24

    25    26    27    28
    29    30    31    32
    25    26    30    29
    26    27    31    30
    27    28    32    31
    28    25    29    32

    33    34    35    36
    37    38    39    40
    33    34    38    37
    34    35    39    38
    35    36    40    39
    36    33    37    40
    ];

T3_org = [T3_org'; ones(1,size(T3_org,1))];
T3_org = trans(825+DT,0,0)*T3_org;
T3_org = T3_org(1:3,:)';

T3 = patch('Vertices',T3_org,'Faces',T3_Faces,'FaceColor',"#02613e");
%%%%%%%%%% T3 %%%%%%%%%%

%%%%%%%%%% R_Base %%%%%%%%%%
R_Base_org = [0 0 0
    0 250 0
    250 250 0
    250 0 0
    0 0 HT
    0 250 HT
    250 250 HT
    250 0 HT];

R_Base_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];

R_Base_org = [R_Base_org'; ones(1,size(R_Base_org,1))];
R_Base_org = trans(905,1250+DT2-125,0)*R_Base_org;
R_Base_org = R_Base_org(1:3,:)';

R_BASE = patch('Vertices',R_Base_org,'Faces',R_Base_Faces,'FaceColor',"#535454");
%%%%%%%%%% R_Base %%%%%%%%%%

%%%%%%%%%% DH_Matrix %%%%%%%%%%
% DH matriz
% TH - ANGULO DE ROTAÇÃO EM Z
% L DESLOCAMENTO EM X
% D - DESLOCAMENTO EM Z
% AL ROTAÇÃO EM X
DH_ur10e = [0 0 LA 90
    0 LC 0 0
    0 LE 0 0
    0 0 LF 90
    0 0 LG -90
    0 0 LH 0];

% joints angle
j1 = -90;
j2 = -90;
j3 = 0;
j4 = -90;
j5 = 180;
j6 = 0;

% init robot pose
QQ = [j1 j2 j3 j4 j5 j6;j1 j2 j3 j4 j5 j6]';
steps = 50;
jTypes = [0 0 0 0 0 0];
sScale = 50;
robot_pose = [1030 1250+DT2-125+125 HT];
[H,h,~,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
%%%%%%%%%% DH_Matrix %%%%%%%%%%
% to record movie
% pause(15)

if draw_end_path == 1
    pause(1)

    %%%%%%%%%%%%%%%%%%%%%%%%%%% DRAw %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
    steps = 100;
    expected_pose = [0 900 0];
    h_C2_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C2_org;
    red = 1;
    pose_x = h_C2_exp(1,19)-robot_pose(1);
    pose_y = h_C2_exp(2,19)-robot_pose(2)+100+(DT2-370);
    pose_z = HT+25-robot_pose(3);
    phi_z = 1;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    QQ = [j1 j2 j3 j4 j5 j6
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    Q0 = QQ(:,1);
    Qf = QQ(:,2);
    t0 = 0;
    tf = 2;
    [QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
        [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
        objs=cat(3,C1,C2,C3);
        h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
        pos=[(-(((905+125+DL-160)-1030+50)-DA))/steps 0 0 0 0 0;0 900/steps 0 0 0 0;0 0 0 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C1_org = objs_final_pos(:,:,1);
        h_C2_org = objs_final_pos(:,:,2);
        h_C3_org = objs_final_pos(:,:,3);
    end

    % grasp the part with jacobian (GRASPING)
    jacob_moves = [0 -150-(DT2-405) 0];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    Qn = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)]';
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        Qn = Qn+dQ;
    end

    % pull part from the table IN Z AXIS
    Qn = Qn-dQ;
    q = Qn';
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    jacob_moves = [0 0 200];
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[0 0 200/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % Rotate to main part
    x = x1;
    y = y1;
    z = z1;
    pose_x = h_C1_org(1,15)-robot_pose(1);
    pose_y = h_C1_org(2,15)-robot_pose(2)-100;
    pose_z = h_C1_org(3,15)-robot_pose(3)+50;
    phi_z = 91;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) Q_inv1(6,red)
        ]';
    dQ = (QQ(:,2) - QQ(:,1))/steps;
    Qn = QQ(:,1);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        part_pose = DrawHandPath(AAA,robot_pose,1,h_C2_org_ori);
        Qn = Qn+dQ;
    end
    h_C2_org = part_pose;

    % move to main part X,Y AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [h_C1_org(1,15)-h_C2_org(1,23) h_C1_org(2,15)-h_C2_org(2,23) 0];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % move to main part Z AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 0 -(h_C2_org(3,19)-h_C1_org(3,5))];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % drop part and move to second part
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [-300 0 0];
    steps = 50;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        Qn = Qn+dQ;
    end
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%

    %%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%
    steps = 100;
    red = 1;
    Qn = Qn-dQ;
    q = Qn';
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    expected_pose = [0 960 0];
    h_C3_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C3_org;
    pose_x = h_C3_exp(1,33)-robot_pose(1)+60;
    pose_y = h_C3_exp(2,33)-robot_pose(2);
    pose_z = HT+25-robot_pose(3);
    phi_z = 1;
    theta_y = -91;
    psi_x = 1;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)];
    Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
    red2 = 1;
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv1(1,red2) Q_inv1(2,red2) Q_inv1(3,red2) Q_inv1(4,red2) Q_inv1(5,red2) 0
        ]';
    Q0 = QQ(:,1);
    Qf = QQ(:,2);
    t0 = 0;
    tf = 2;
    [QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
        [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
        objs=cat(3,C1,C2,C3);
        h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
        pos=[0 0 0 0 0 0;0 0 0 0 0 0;0 960/steps 0 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C1_org = objs_final_pos(:,:,1);
        h_C2_org = objs_final_pos(:,:,2);
        h_C3_org = objs_final_pos(:,:,3);
    end

    % approach part 2
    q = [Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) 0];
    Qn = q';
    jacob_moves = [-80 0 0];
    steps = 50;
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        Qn = Qn+dQ;
    end

    % move part2 apart from small T
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 0 250];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % Rotate to main part
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        if n>=2
            objs=cat(3,C3);
            h_objs=cat(3,h_C3_org);
            pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
            N = 1;
            objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
            % update positions
            h_C3_org = objs_final_pos(:,:,1);
        end
        objs=cat(3,C1,C2);
        h_objs=cat(3,h_C1_org,h_C2_org);
        pos=[-(DA+DB)/steps 0 0 0 0 0;-(DA+DB)/steps 0 0 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C1_org = objs_final_pos(:,:,1);
        h_C2_org = objs_final_pos(:,:,2);
        Qn = Qn+dQ;
    end

    % move to main part X,Y AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [(h_C1_org(1,15)-robot_pose(1))-(h_C3_org(1,32)-robot_pose(1)) (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        if n>=2
            objs=cat(3,C3);
            h_objs=cat(3,h_C3_org);
            pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
            N = 1;
            objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
            % update positions
            h_C3_org = objs_final_pos(:,:,1);
        end
        Qn = Qn+dQ;
    end

    % move to main part Z AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 0 -(h_C3_org(3,10)-h_C2_org(3,1))];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult_Draw(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % drop part and move
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [200 0 0];
    steps = 50;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        DrawHandPath(AAA,robot_pose);
        Qn = Qn+dQ;
    end
    %%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%% DRAW %%%%%%%%%%%%%%%%%%%%%

    pause(2)
end
clf
hold on
axis off
grid on
view(120,20)
axis_dim = 3000;
axis image
axis([-50 axis_dim -50 axis_dim 0 2200])
% axis square
xlabel('x')
ylabel('y')
zlabel('z')

%%%%%%%%%% C1 %%%%%%%%%%
C1_org = [0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 50
    0 100 50
    100 100 50
    100 0 50
    50 26 50
    74 50 50
    50 74 50
    26 50 50
    50 26 200
    74 50 200
    50 74 200
    26 50 200];

random_pose = random_pose_draw_C1;
h_C1_org = [C1_org'; ones(1,size(C1_org,1))];
h_C1_org = trans(905+125+DL-160,1250+DT2-125+125+DT1+random_pose,HT)*h_C1_org;
C1_org = h_C1_org(1:3,:)';

C1_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
    9 10 11 12
    13 14 15 16
    9 10 14 13
    10 11 15 14
    11 12 16 15
    12 9 13 16];

C1 = patch('Vertices',C1_org,'Faces',C1_Faces,'FaceColor',"#535454");
%%%%%%%%%% C1 %%%%%%%%%%

%%%%%%%%%% C2 %%%%%%%%%%
C2_org = [0 0 0
    0 35 0
    11 35 0
    35 11 0
    35 0 0
    0 0 50
    0 35 50
    11 35 50
    35 11 50
    35 0 50

    0 35 0
    11 35 0
    35 59 0
    35 70 0
    0 70 0
    0 35 50
    11 35 50
    35 59 50
    35 70 50
    0 70 50

    35 0 0
    35 11 0
    59 35 0
    70 35 0
    70 0 0
    35 0 50
    35 11 50
    59 35 50
    70 35 50
    70 0 50

    35 70 0
    35 59 0
    59 35 0
    70 35 0
    70 70 0
    35 70 50
    35 59 50
    59 35 50
    70 35 50
    70 70 50
    ];

h_C2_org = [C2_org'; ones(1,size(C2_org,1))];
h_C2_org_ori = h_C2_org;
h_C2_org_ori = rotx(90)*h_C2_org_ori;
h_C2_org_ori = trans(-35,25,-13)*h_C2_org_ori;
random_pose = random_pose_draw_C2;
h_C2_org = trans(1300+random_pose+(DT-410),200,HT)*h_C2_org;
C2_org = h_C2_org(1:3,:)';

C2_Faces = [1 2 3 4 5
    6 7 8 9 10
    1 2 2 7 6
    2 3 3 8 7
    3 4 4 9 8
    4 5 5 10 9
    5 1 1 6 10

    11 12 13 14 15
    16 17 18 19 20
    11 12 12 17 16
    12 13 13 18 17
    13 14 14 19 18
    14 15 15 20 19
    15 11 11 16 20

    21    22    23    24    25
    26    27    28    29    30
    21    22    22    27    26
    22    23    23    28    27
    23    24    24    29    28
    24    25    25    30    29
    25    21    21    26    30

    31    32    33    34    35
    36    37    38    39    40
    31    32    32    37    36
    32    33    33    38    37
    33    34    34    39    38
    34    35    35    40    39
    35    31    31    36    40
    ];

C2 = patch('Vertices',C2_org,'Faces',C2_Faces,'FaceColor',"#535454");
%%%%%%%%%% C2 %%%%%%%%%%

%%%%%%%%%% C3 %%%%%%%%%%
t = 0.05:0.5:2*pi;
x1 = 30*cos(t);
y1 = 30*sin(t);
C3_1 = [x1' y1' zeros(size(x1))'];
C3_2 = C3_1;
C3_2(:,3) = 50;
C3_org = [C3_1;C3_2

-24 0 0
0 24 0
24 0 0
0 -24 0
-24 0 50
0 24 50
24 0 50
0 -24 50];

h_C3_org = [C3_org'; ones(1,size(C3_org,1))];
h_C3_org_ori = h_C3_org;
h_C3_org_ori = rotx(90)*h_C3_org_ori;
h_C3_org_ori = trans(0,25,18)*h_C3_org_ori;
random_pose = random_pose_draw_C3;
h_C3_org = trans(300+random_pose,230,HT)*h_C3_org;
C3_org = h_C3_org(1:3,:)';

C3_Faces = [1 2 3 4 5 6 7 8 9 10 11 12 13
    14 15 16 17 18 19 20 21 22 23 24 25 26
    1 2 15 15 15 15 15 15 15 15 15 15 14
    2 3 16 16 16 16 16 16 16 16 16 16 15
    3 4 17 17 17 17 17 17 17 17 17 17 16
    4 5 18 18 18 18 18 18 18 18 18 18 17
    5 6 19 19 19 19 19 19 19 19 19 19 18
    6 7 20 20 20 20 20 20 20 20 20 20 19
    7 8 21 21 21 21 21 21 21 21 21 21 20
    8 9 22 22 22 22 22 22 22 22 22 22 21
    9 10 23 23 23 23 23 23 23 23 23 23 22
    10 11 24 24 24 24 24 24 24 24 24 24 23
    11 12 25 25 25 25 25 25 25 25 25 25 24
    12 13 26 26 26 26 26 26 26 26 26 26 25
    13 1 1 1 1 1 1 1 1 1 1 14 26

    27 28 28 28 28 28 28 28 28 28 28 29 30
    31 32 33 33 33 33 33 33 33 33 33 33 34
    27 31 32 32 32 32 32 32 32 32 32 32 28
    28 32 33 33 33 33 33 33 33 33 33 33 29
    29 33 34 34 34 34 34 34 34 34 34 34 30
    30 34 27 27 27 27 27 27 27 27 27 27 27
    ];

C3 = patch('Vertices',C3_org,'Faces',C3_Faces,'FaceColor',"#535454");
%%%%%%%%%% C3 %%%%%%%%%%

%%%%%%%%%% T1 %%%%%%%%%%
T1_org = [0 0 HT
    0 2560 HT
    625 2560 HT
    625 0 HT
    0 0 HT-60
    0 2560 HT-60
    625 2560 HT-60
    625 0 HT-60

    0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 HT-60
    0 100 HT-60
    100 100 HT-60
    100 0 HT-60

    625 0 0
    525 0 0
    525 100 0
    625 100 0
    625 0 HT-60
    525 0 HT-60
    525 100 HT-60
    625 100 HT-60

    0 2560 0
    100 2560 0
    100 2460 0
    0 2460 0
    0 2560 HT-60
    100 2560 HT-60
    100 2460 HT-60
    0 2460 HT-60

    625 2560 0
    625 2460 0
    525 2460 0
    525 2560 0
    625 2560 HT-60
    625 2460 HT-60
    525 2460 HT-60
    525 2560 HT-60
    ];

T1_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8

    9    10    11    12
    13    14    15    16
    9    10    14    13
    10    11    15    14
    11    12    16    15
    12     9    13    16

    17    18    19    20
    21    22    23    24
    17    18    22    21
    18    19    23    22
    19    20    24    23
    20    17    21    24

    25    26    27    28
    29    30    31    32
    25    26    30    29
    26    27    31    30
    27    28    32    31
    28    25    29    32

    33    34    35    36
    37    38    39    40
    33    34    38    37
    34    35    39    38
    35    36    40    39
    36    33    37    40
    ];

T1_org = [T1_org'; ones(1,size(T1_org,1))];
T1_org = rotz(90)*T1_org;
T1_org = trans(905+125+DL,1250+DT2-125+125+DT1,0)*T1_org;
T1_org = T1_org(1:3,:)';

T1 = patch('Vertices',T1_org,'Faces',T1_Faces,'FaceColor',"#02613e");
%%%%%%%%%% T1 %%%%%%%%%%

%%%%%%%%%% T2 %%%%%%%%%%
T2_org = [0 0 HT
    0 1250 HT
    625 1250 HT
    625 0 HT
    0 0 HT-60
    0 1250 HT-60
    625 1250 HT-60
    625 0 HT-60

    0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 HT-60
    0 100 HT-60
    100 100 HT-60
    100 0 HT-60

    625 0 0
    525 0 0
    525 100 0
    625 100 0
    625 0 HT-60
    525 0 HT-60
    525 100 HT-60
    625 100 HT-60

    0 1250 0
    100 1250 0
    100 1150 0
    0 1150 0
    0 1250 HT-60
    100 1250 HT-60
    100 1150 HT-60
    0 1150 HT-60

    625 1250 0
    625 1150 0
    525 1150 0
    525 1250 0
    625 1250 HT-60
    625 1150 HT-60
    525 1150 HT-60
    525 1250 HT-60
    ];

T2_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8

    9    10    11    12
    13    14    15    16
    9    10    14    13
    10    11    15    14
    11    12    16    15
    12     9    13    16

    17    18    19    20
    21    22    23    24
    17    18    22    21
    18    19    23    22
    19    20    24    23
    20    17    21    24

    25    26    27    28
    29    30    31    32
    25    26    30    29
    26    27    31    30
    27    28    32    31
    28    25    29    32

    33    34    35    36
    37    38    39    40
    33    34    38    37
    34    35    39    38
    35    36    40    39
    36    33    37    40
    ];

T2_org = [T2_org'; ones(1,size(T2_org,1))];
T2_org = trans(200,0,0)*T2_org;
T2_org = T2_org(1:3,:)';

T2 = patch('Vertices',T2_org,'Faces',T2_Faces,'FaceColor',"#02613e");
%%%%%%%%%% T2 %%%%%%%%%%

%%%%%%%%%% T3 %%%%%%%%%%
T3_org = [0 0 HT
    0 1250 HT
    625 1250 HT
    625 0 HT
    0 0 HT-60
    0 1250 HT-60
    625 1250 HT-60
    625 0 HT-60

    0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 HT-60
    0 100 HT-60
    100 100 HT-60
    100 0 HT-60

    625 0 0
    525 0 0
    525 100 0
    625 100 0
    625 0 HT-60
    525 0 HT-60
    525 100 HT-60
    625 100 HT-60

    0 1250 0
    100 1250 0
    100 1150 0
    0 1150 0
    0 1250 HT-60
    100 1250 HT-60
    100 1150 HT-60
    0 1150 HT-60

    625 1250 0
    625 1150 0
    525 1150 0
    525 1250 0
    625 1250 HT-60
    625 1150 HT-60
    525 1150 HT-60
    525 1250 HT-60
    ];

T3_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8

    9    10    11    12
    13    14    15    16
    9    10    14    13
    10    11    15    14
    11    12    16    15
    12     9    13    16

    17    18    19    20
    21    22    23    24
    17    18    22    21
    18    19    23    22
    19    20    24    23
    20    17    21    24

    25    26    27    28
    29    30    31    32
    25    26    30    29
    26    27    31    30
    27    28    32    31
    28    25    29    32

    33    34    35    36
    37    38    39    40
    33    34    38    37
    34    35    39    38
    35    36    40    39
    36    33    37    40
    ];

T3_org = [T3_org'; ones(1,size(T3_org,1))];
T3_org = trans(825+DT,0,0)*T3_org;
T3_org = T3_org(1:3,:)';

T3 = patch('Vertices',T3_org,'Faces',T3_Faces,'FaceColor',"#02613e");
%%%%%%%%%% T3 %%%%%%%%%%

%%%%%%%%%% R_Base %%%%%%%%%%
R_Base_org = [0 0 0
    0 250 0
    250 250 0
    250 0 0
    0 0 HT
    0 250 HT
    250 250 HT
    250 0 HT];

R_Base_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];

R_Base_org = [R_Base_org'; ones(1,size(R_Base_org,1))];
R_Base_org = trans(905,1250+DT2-125,0)*R_Base_org;
R_Base_org = R_Base_org(1:3,:)';

R_BASE = patch('Vertices',R_Base_org,'Faces',R_Base_Faces,'FaceColor',"#535454");
%%%%%%%%%% R_Base %%%%%%%%%%

%%%%%%%%%% DH_Matrix %%%%%%%%%%
% DH matriz
% TH - ANGULO DE ROTAÇÃO EM Z
% L DESLOCAMENTO EM X
% D - DESLOCAMENTO EM Z
% AL ROTAÇÃO EM X
DH_ur10e = [0 0 LA 90
    0 LC 0 0
    0 LE 0 0
    0 0 LF 90
    0 0 LG -90
    0 0 LH 0];

% joints angle
j1 = -90;
j2 = -90;
j3 = 0;
j4 = -90;
j5 = 180;
j6 = 0;

% init robot pose
QQ = [j1 j2 j3 j4 j5 j6;j1 j2 j3 j4 j5 j6]';
steps = 50;
jTypes = [0 0 0 0 0 0];
sScale = 50;
robot_pose = [1030 1250+DT2-125+125 HT];
[H,h,~,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
%%%%%%%%%% DH_Matrix %%%%%%%%%%

plot_time = 0;

if debug_mode == 0
    %%%%%%%%%% MOVE PARTS %%%%%%%%%%
    if debug_mode == 0
        pause(1)
    end
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
    steps = 100;
    expected_pose = [0 900 0];
    h_C2_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C2_org;
    red = 1;
    pose_x = h_C2_exp(1,19)-robot_pose(1);
    pose_y = h_C2_exp(2,19)-robot_pose(2)+100+(DT2-370);
    pose_z = HT+25-robot_pose(3);
    phi_z = 1;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    QQ = [j1 j2 j3 j4 j5 j6
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    Q0 = QQ(:,1);
    Qf = QQ(:,2);
    t0 = 0;
    tf = 2;
    [QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
        [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
        objs=cat(3,C1,C2,C3);
        h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
        pos=[(-(((905+125+DL-160)-1030+50)-DA))/steps 0 0 0 0 0;0 900/steps 0 0 0 0;0 0 0 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C1_org = objs_final_pos(:,:,1);
        h_C2_org = objs_final_pos(:,:,2);
        h_C3_org = objs_final_pos(:,:,3);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,QQ(1,n),'.r');
            plot(plot_time,QQ(2,n),'.b');
            plot(plot_time,QQ(3,n),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
    end

    figure(1)
    if debug_mode == 0
        for n = 120:-1:100
            view(n,20)
            pause(0.01)
        end
    end

    % grasp the part with jacobian (GRASPING)
    if debug_mode == 0
        pause(0.3)
    end
    jacob_moves = [0 -150-(DT2-405) 0];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    Qn = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)]';
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,Qn(1),'.r');
            plot(plot_time,Qn(2),'.b');
            plot(plot_time,Qn(3),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
        Qn = Qn+dQ;
    end

    figure(1)
    % pull part from the table IN Z AXIS
    if debug_mode == 0
        pause(0.3)
    end
    Qn = Qn-dQ;
    q = Qn';
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    jacob_moves = [0 0 200];
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[0 0 200/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,Qn(1),'.r');
            plot(plot_time,Qn(2),'.b');
            plot(plot_time,Qn(3),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
        Qn = Qn+dQ;
    end

    % Rotate to main part
    figure(1)
    x = x1;
    y = y1;
    z = z1;
    pose_x = h_C1_org(1,15)-robot_pose(1);
    pose_y = h_C1_org(2,15)-robot_pose(2)-100;
    pose_z = h_C1_org(3,15)-robot_pose(3)+50;
    phi_z = 91;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) Q_inv1(6,red)
        ]';
    dQ = (QQ(:,2) - QQ(:,1))/steps;
    Qn = QQ(:,1);
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        part_pose = AnimateRobot(H,AAA,P,h,0.01,0,robot_pose,1,h_C2_org_ori,C2);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,Qn(1),'.r');
            plot(plot_time,Qn(2),'.b');
            plot(plot_time,Qn(3),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
        Qn = Qn+dQ;
    end
    h_C2_org = part_pose;

    figure(1)
    if debug_mode == 0
        for n = 100:140
            view(n,n-80)
            pause(0.01)
        end
    end

    % move to main part X,Y AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [h_C1_org(1,15)-h_C2_org(1,23) h_C1_org(2,15)-h_C2_org(2,23) 0];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,Qn(1),'.r');
            plot(plot_time,Qn(2),'.b');
            plot(plot_time,Qn(3),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
        Qn = Qn+dQ;
    end

    figure(1)
    % move to main part Z AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 0 -(h_C2_org(3,19)-h_C1_org(3,5))];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        if n == 20
            figure(1)
            view(90,90) % x,y
        end
        if n == 80
            figure(1)
            view(120,20)
        end
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,Qn(1),'.r');
            plot(plot_time,Qn(2),'.b');
            plot(plot_time,Qn(3),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
        Qn = Qn+dQ;
    end

    figure(1)
    % drop part and move to second part
    if debug_mode == 0
        pause(0.3)
    end
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [-300 0 0];
    steps = 50;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        if plot_joints == 1
            figure(2);
            axis square;
            hold on;
            grid on;
            xlabel('ite/time');
            ylabel('deg');
            plot_time = plot_time + 1;
            plot(plot_time,Qn(1),'.r');
            plot(plot_time,Qn(2),'.b');
            plot(plot_time,Qn(3),'.y');
            hold off
            legend('theta1','theta2','theta3');
        end
        Qn = Qn+dQ;
    end
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
end

figure(1)
view(220,20)

%%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%
steps = 100;
red = 1;
Qn = Qn-dQ;
q = Qn';
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
expected_pose = [0 960 0];
h_C3_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C3_org;
pose_x = h_C3_exp(1,33)-robot_pose(1)+60;
pose_y = h_C3_exp(2,33)-robot_pose(2);
pose_z = HT+25-robot_pose(3);
phi_z = 1;
theta_y = -91;
psi_x = 1;
M_rot = Trot(phi_z,theta_y,psi_x);
TT = M_rot;
TT(1,4) = pose_x;
TT(2,4) = pose_y;
TT(3,4) = pose_z;
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x1 = [TT(1,1:3) TT(1,4)];
y1 = [TT(2,1:3) TT(2,4)];
z1 = [TT(3,1:3) TT(3,4)];
Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
red2 = 1;
QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
    Q_inv1(1,red2) Q_inv1(2,red2) Q_inv1(3,red2) Q_inv1(4,red2) Q_inv1(5,red2) 0
    ]';
Q0 = QQ(:,1);
Qf = QQ(:,2);
t0 = 0;
tf = 2;
[QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
    [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
    objs=cat(3,C1,C2,C3);
    h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
    pos=[0 0 0 0 0 0;0 0 0 0 0 0;0 960/steps 0 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C1_org = objs_final_pos(:,:,1);
    h_C2_org = objs_final_pos(:,:,2);
    h_C3_org = objs_final_pos(:,:,3);
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,QQ(1,n),'.r');
        plot(plot_time,QQ(2,n),'.b');
        plot(plot_time,QQ(3,n),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
end

figure(1)
% approach part 2
if debug_mode == 0
    pause(0.3)
end
q = [Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) 0];
Qn = q';
jacob_moves = [-80 0 0];
steps = 50;
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,Qn(1),'.r');
        plot(plot_time,Qn(2),'.b');
        plot(plot_time,Qn(3),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
    Qn = Qn+dQ;
end

figure(1)
% move part2 apart from small T
if debug_mode == 0
    pause(0.3)
end
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 0 250];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    objs=cat(3,C3);
    h_objs=cat(3,h_C3_org);
    pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C3_org = objs_final_pos(:,:,1);
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,Qn(1),'.r');
        plot(plot_time,Qn(2),'.b');
        plot(plot_time,Qn(3),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
    Qn = Qn+dQ;
end

figure(1)
% Rotate to main part
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if n>=2
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
    end
    objs=cat(3,C1,C2);
    h_objs=cat(3,h_C1_org,h_C2_org);
    pos=[-(DA+DB)/steps 0 0 0 0 0;-(DA+DB)/steps 0 0 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C1_org = objs_final_pos(:,:,1);
    h_C2_org = objs_final_pos(:,:,2);
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,Qn(1),'.r');
        plot(plot_time,Qn(2),'.b');
        plot(plot_time,Qn(3),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
    Qn = Qn+dQ;
end

figure(1)
% move to main part X,Y AXIS
Qn = Qn-dQ;
q = Qn';
jacob_moves = [(h_C1_org(1,15)-robot_pose(1))-(h_C3_org(1,32)-robot_pose(1)) (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if n>=2
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
    end
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,Qn(1),'.r');
        plot(plot_time,Qn(2),'.b');
        plot(plot_time,Qn(3),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
    Qn = Qn+dQ;
end

figure(1)
if debug_mode == 0
    for n = 220:-1:150
        view(n,20)
        pause(0.01)
    end
end

% move to main part Z AXIS
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 0 -(h_C3_org(3,10)-h_C2_org(3,1))];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    if n == 20
        figure(1)
        view(90,90) % x,y
    end
    if n == 80
        figure(1)
        view(120,20)
    end
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    objs=cat(3,C3);
    h_objs=cat(3,h_C3_org);
    pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C3_org = objs_final_pos(:,:,1);
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,Qn(1),'.r');
        plot(plot_time,Qn(2),'.b');
        plot(plot_time,Qn(3),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
    Qn = Qn+dQ;
end

figure(1)
% drop part and move
if debug_mode == 0
    pause(0.3)
end
Qn = Qn-dQ;
q = Qn';
jacob_moves = [200 0 0];
steps = 50;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if plot_joints == 1
        figure(2);
        axis square;
        hold on;
        grid on;
        xlabel('ite/time');
        ylabel('deg');
        plot_time = plot_time + 1;
        plot(plot_time,Qn(1),'.r');
        plot(plot_time,Qn(2),'.b');
        plot(plot_time,Qn(3),'.y');
        hold off
        legend('theta1','theta2','theta3');
    end
    Qn = Qn+dQ;
end
figure(1)
%%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%

%%%%%%%%%% MOVE PARTS 3 %%%%%%%%%%
[h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
[h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);

objs=cat(3,C1,C2,C3);
h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
pos=[-890 0 0 0 0 0;-890 0 0 0 0 0;-890 0 0 0 0 0];
steps = 50;
objs_final_pos = Move_Simult(objs,h_objs,steps,pos);
% update positions
h_C1_org = objs_final_pos(:,:,1);
h_C2_org = objs_final_pos(:,:,2);
h_C3_org = objs_final_pos(:,:,3);
%%%%%%%%%% MOVE PARTS 3 %%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% C1 %%%%%%%%%%
C1_org = [0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 50
    0 100 50
    100 100 50
    100 0 50
    50 26 50
    74 50 50
    50 74 50
    26 50 50
    50 26 200
    74 50 200
    50 74 200
    26 50 200];

random_pose = rand(1) * 500;
h_C1_org = [C1_org'; ones(1,size(C1_org,1))];
h_C1_org = trans(905+125+DL-160,1250+DT2-125+125+DT1+random_pose,HT)*h_C1_org;
C1_org = h_C1_org(1:3,:)';

C1_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
    9 10 11 12
    13 14 15 16
    9 10 14 13
    10 11 15 14
    11 12 16 15
    12 9 13 16];

C1 = patch('Vertices',C1_org,'Faces',C1_Faces,'FaceColor',"#535454");
%%%%%%%%%% C1 %%%%%%%%%%

%%%%%%%%%% C2 %%%%%%%%%%
C2_org = [0 0 0
    0 35 0
    11 35 0
    35 11 0
    35 0 0
    0 0 50
    0 35 50
    11 35 50
    35 11 50
    35 0 50

    0 35 0
    11 35 0
    35 59 0
    35 70 0
    0 70 0
    0 35 50
    11 35 50
    35 59 50
    35 70 50
    0 70 50

    35 0 0
    35 11 0
    59 35 0
    70 35 0
    70 0 0
    35 0 50
    35 11 50
    59 35 50
    70 35 50
    70 0 50

    35 70 0
    35 59 0
    59 35 0
    70 35 0
    70 70 0
    35 70 50
    35 59 50
    59 35 50
    70 35 50
    70 70 50
    ];

h_C2_org = [C2_org'; ones(1,size(C2_org,1))];
h_C2_org_ori = h_C2_org;
h_C2_org_ori = rotx(90)*h_C2_org_ori;
h_C2_org_ori = trans(-35,25,-13)*h_C2_org_ori;
random_pose = rand(1) * 500;
h_C2_org = trans(1300+random_pose+(DT-410),200,HT)*h_C2_org;
C2_org = h_C2_org(1:3,:)';

C2_Faces = [1 2 3 4 5
    6 7 8 9 10
    1 2 2 7 6
    2 3 3 8 7
    3 4 4 9 8
    4 5 5 10 9
    5 1 1 6 10

    11 12 13 14 15
    16 17 18 19 20
    11 12 12 17 16
    12 13 13 18 17
    13 14 14 19 18
    14 15 15 20 19
    15 11 11 16 20

    21    22    23    24    25
    26    27    28    29    30
    21    22    22    27    26
    22    23    23    28    27
    23    24    24    29    28
    24    25    25    30    29
    25    21    21    26    30

    31    32    33    34    35
    36    37    38    39    40
    31    32    32    37    36
    32    33    33    38    37
    33    34    34    39    38
    34    35    35    40    39
    35    31    31    36    40
    ];

C2 = patch('Vertices',C2_org,'Faces',C2_Faces,'FaceColor',"#535454");
%%%%%%%%%% C2 %%%%%%%%%%

%%%%%%%%%% C3 %%%%%%%%%%
t = 0.05:0.5:2*pi;
x1 = 30*cos(t);
y1 = 30*sin(t);
C3_1 = [x1' y1' zeros(size(x1))'];
C3_2 = C3_1;
C3_2(:,3) = 50;
C3_org = [C3_1;C3_2

-24 0 0
0 24 0
24 0 0
0 -24 0
-24 0 50
0 24 50
24 0 50
0 -24 50];

h_C3_org = [C3_org'; ones(1,size(C3_org,1))];
h_C3_org_ori = h_C3_org;
h_C3_org_ori = rotx(90)*h_C3_org_ori;
h_C3_org_ori = trans(0,25,18)*h_C3_org_ori;
random_pose = rand(1) * 300;
h_C3_org = trans(300+random_pose,230,HT)*h_C3_org;
C3_org = h_C3_org(1:3,:)';

C3_Faces = [1 2 3 4 5 6 7 8 9 10 11 12 13
    14 15 16 17 18 19 20 21 22 23 24 25 26
    1 2 15 15 15 15 15 15 15 15 15 15 14
    2 3 16 16 16 16 16 16 16 16 16 16 15
    3 4 17 17 17 17 17 17 17 17 17 17 16
    4 5 18 18 18 18 18 18 18 18 18 18 17
    5 6 19 19 19 19 19 19 19 19 19 19 18
    6 7 20 20 20 20 20 20 20 20 20 20 19
    7 8 21 21 21 21 21 21 21 21 21 21 20
    8 9 22 22 22 22 22 22 22 22 22 22 21
    9 10 23 23 23 23 23 23 23 23 23 23 22
    10 11 24 24 24 24 24 24 24 24 24 24 23
    11 12 25 25 25 25 25 25 25 25 25 25 24
    12 13 26 26 26 26 26 26 26 26 26 26 25
    13 1 1 1 1 1 1 1 1 1 1 14 26

    27 28 28 28 28 28 28 28 28 28 28 29 30
    31 32 33 33 33 33 33 33 33 33 33 33 34
    27 31 32 32 32 32 32 32 32 32 32 32 28
    28 32 33 33 33 33 33 33 33 33 33 33 29
    29 33 34 34 34 34 34 34 34 34 34 34 30
    30 34 27 27 27 27 27 27 27 27 27 27 27
    ];

C3 = patch('Vertices',C3_org,'Faces',C3_Faces,'FaceColor',"#535454");
%%%%%%%%%% C3 %%%%%%%%%%

if debug_mode == 0
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
    Qn = Qn-dQ;
    steps = 100;
    expected_pose = [0 900 0];
    h_C2_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C2_org;
    red = 1;
    pose_x = h_C2_exp(1,19)-robot_pose(1);
    pose_y = h_C2_exp(2,19)-robot_pose(2)+100+(DT2-370);
    pose_z = HT+25-robot_pose(3);
    phi_z = 1;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    QQ = [Qn'
        -j1 j2 90 -j4 100 j6
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    Q0 = QQ(:,1);
    Qf = QQ(:,3);
    t0 = 0;
    tf = 2;
    t = [0 1 2];
    %     [QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
    [QQ,~] = MultiPolyTrajV(QQ,steps,t,1);
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:size(QQ,2)
        MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
        [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
        objs=cat(3,C1,C2,C3);
        h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
        pos=[(-(((905+125+DL-160)-1030+50)-DA))/size(QQ,2)  0 0 0 0 0;0 900/size(QQ,2) 0 0 0 0;0 0 0 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C1_org = objs_final_pos(:,:,1);
        h_C2_org = objs_final_pos(:,:,2);
        h_C3_org = objs_final_pos(:,:,3);
    end

    if debug_mode == 0
        for n = 120:-1:100
            view(n,20)
            pause(0.01)
        end
    end

    % grasp the part with jacobian (GRASPING)
    if debug_mode == 0
        pause(0.3)
    end
    jacob_moves = [0 -150-(DT2-405) 0];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    Qn = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)]';
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
        Qn = Qn+dQ;
    end

    % pull part from the table IN Z AXIS
    if debug_mode == 0
        pause(0.3)
    end
    Qn = Qn-dQ;
    q = Qn';
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    jacob_moves = [0 0 200];
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[0 0 200/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % Rotate to main part
    x = x1;
    y = y1;
    z = z1;
    pose_x = h_C1_org(1,15)-robot_pose(1);
    pose_y = h_C1_org(2,15)-robot_pose(2)-100;
    pose_z = h_C1_org(3,15)-robot_pose(3)+50;
    phi_z = 91;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) Q_inv1(6,red)
        ]';
    dQ = (QQ(:,2) - QQ(:,1))/steps;
    Qn = QQ(:,1);
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        part_pose = AnimateRobot(H,AAA,P,h,0.01,0,robot_pose,1,h_C2_org_ori,C2);
        Qn = Qn+dQ;
    end
    h_C2_org = part_pose;

    if debug_mode == 0
        for n = 100:140
            view(n,n-80)
            pause(0.01)
        end
    end

    % move to main part X,Y AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [h_C1_org(1,15)-h_C2_org(1,23) h_C1_org(2,15)-h_C2_org(2,23) 0];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % move to main part Z AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 0 -(h_C2_org(3,19)-h_C1_org(3,5))];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        if n == 20
            view(90,90) % x,y
        end
        if n == 80
            view(120,20)
        end
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % drop part and move to second part
    if debug_mode == 0
        pause(0.3)
    end
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [-300 0 0];
    steps = 50;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        Qn = Qn+dQ;
    end
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
    view(220,20)
end

%%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%
steps = 100;
red = 1;
Qn = Qn-dQ;
q = Qn';
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
expected_pose = [0 960 0];
h_C3_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C3_org;
pose_x = h_C3_exp(1,33)-robot_pose(1)+60;
pose_y = h_C3_exp(2,33)-robot_pose(2);
pose_z = HT+25-robot_pose(3);
phi_z = 1;
theta_y = -91;
psi_x = 1;
M_rot = Trot(phi_z,theta_y,psi_x);
TT = M_rot;
TT(1,4) = pose_x;
TT(2,4) = pose_y;
TT(3,4) = pose_z;
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x1 = [TT(1,1:3) TT(1,4)];
y1 = [TT(2,1:3) TT(2,4)];
z1 = [TT(3,1:3) TT(3,4)];
Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
red2 = 1;
QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
    Q_inv1(1,red2) Q_inv1(2,red2) Q_inv1(3,red2) Q_inv1(4,red2) Q_inv1(5,red2) 0
    ]';
Q0 = QQ(:,1);
Qf = QQ(:,2);
t0 = 0;
tf = 2;
[QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
    [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
    objs=cat(3,C1,C2,C3);
    h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
    pos=[0 0 0 0 0 0;0 0 0 0 0 0;0 960/steps 0 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C1_org = objs_final_pos(:,:,1);
    h_C2_org = objs_final_pos(:,:,2);
    h_C3_org = objs_final_pos(:,:,3);
end

% approach part 2
if debug_mode == 0
    pause(0.3)
end
q = [Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) 0];
Qn = q';
jacob_moves = [-80 0 0];
steps = 50;
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
    Qn = Qn+dQ;
end

% move part2 apart from small T
if debug_mode == 0
    pause(0.3)
end
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 0 250];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    objs=cat(3,C3);
    h_objs=cat(3,h_C3_org);
    pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C3_org = objs_final_pos(:,:,1);
    Qn = Qn+dQ;
end

% Rotate to main part
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if n>=2
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
    end
    objs=cat(3,C1,C2);
    h_objs=cat(3,h_C1_org,h_C2_org);
    pos=[-(DA+DB)/steps 0 0 0 0 0;-(DA+DB)/steps 0 0 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C1_org = objs_final_pos(:,:,1);
    h_C2_org = objs_final_pos(:,:,2);
    Qn = Qn+dQ;
end

% move to main part X,Y AXIS
Qn = Qn-dQ;
q = Qn';
jacob_moves = [(h_C1_org(1,15)-robot_pose(1))-(h_C3_org(1,32)-robot_pose(1)) (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if n>=2
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
    end
    Qn = Qn+dQ;
end

if debug_mode == 0
    for n = 220:-1:150
        view(n,20)
        pause(0.01)
    end
end

% move to main part Z AXIS
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 0 -(h_C3_org(3,10)-h_C2_org(3,1))];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    if n == 20
        view(90,90) % x,y
    end
    if n == 80
        view(120,20)
    end
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    objs=cat(3,C3);
    h_objs=cat(3,h_C3_org);
    pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C3_org = objs_final_pos(:,:,1);
    Qn = Qn+dQ;
end

% drop part and move
if debug_mode == 0
    pause(0.3)
end
Qn = Qn-dQ;
q = Qn';
jacob_moves = [200 0 0];
steps = 50;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    Qn = Qn+dQ;
end
%%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%

%%%%%%%%%% MOVE PARTS 3 %%%%%%%%%%
[h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
[h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);

objs=cat(3,C1,C2,C3);
h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
pos=[-890 0 0 0 0 0;-890 0 0 0 0 0;-890 0 0 0 0 0];
steps = 50;
objs_final_pos = Move_Simult(objs,h_objs,steps,pos);
% update positions
h_C1_org = objs_final_pos(:,:,1);
h_C2_org = objs_final_pos(:,:,2);
h_C3_org = objs_final_pos(:,:,3);
%%%%%%%%%% MOVE PARTS 3 %%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%% C1 %%%%%%%%%%
C1_org = [0 0 0
    0 100 0
    100 100 0
    100 0 0
    0 0 50
    0 100 50
    100 100 50
    100 0 50
    50 26 50
    74 50 50
    50 74 50
    26 50 50
    50 26 200
    74 50 200
    50 74 200
    26 50 200];

random_pose = rand(1) * 500;
h_C1_org = [C1_org'; ones(1,size(C1_org,1))];
h_C1_org = trans(905+125+DL-160,1250+DT2-125+125+DT1+random_pose,HT)*h_C1_org;
C1_org = h_C1_org(1:3,:)';

C1_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8
    9 10 11 12
    13 14 15 16
    9 10 14 13
    10 11 15 14
    11 12 16 15
    12 9 13 16];

C1 = patch('Vertices',C1_org,'Faces',C1_Faces,'FaceColor',"#535454");
%%%%%%%%%% C1 %%%%%%%%%%

%%%%%%%%%% C2 %%%%%%%%%%
C2_org = [0 0 0
    0 35 0
    11 35 0
    35 11 0
    35 0 0
    0 0 50
    0 35 50
    11 35 50
    35 11 50
    35 0 50

    0 35 0
    11 35 0
    35 59 0
    35 70 0
    0 70 0
    0 35 50
    11 35 50
    35 59 50
    35 70 50
    0 70 50

    35 0 0
    35 11 0
    59 35 0
    70 35 0
    70 0 0
    35 0 50
    35 11 50
    59 35 50
    70 35 50
    70 0 50

    35 70 0
    35 59 0
    59 35 0
    70 35 0
    70 70 0
    35 70 50
    35 59 50
    59 35 50
    70 35 50
    70 70 50
    ];

h_C2_org = [C2_org'; ones(1,size(C2_org,1))];
h_C2_org_ori = h_C2_org;
h_C2_org_ori = rotx(90)*h_C2_org_ori;
h_C2_org_ori = trans(-35,25,-13)*h_C2_org_ori;
random_pose = rand(1) * 500;
h_C2_org = trans(1300+random_pose+(DT-410),200,HT)*h_C2_org;
C2_org = h_C2_org(1:3,:)';

C2_Faces = [1 2 3 4 5
    6 7 8 9 10
    1 2 2 7 6
    2 3 3 8 7
    3 4 4 9 8
    4 5 5 10 9
    5 1 1 6 10

    11 12 13 14 15
    16 17 18 19 20
    11 12 12 17 16
    12 13 13 18 17
    13 14 14 19 18
    14 15 15 20 19
    15 11 11 16 20

    21    22    23    24    25
    26    27    28    29    30
    21    22    22    27    26
    22    23    23    28    27
    23    24    24    29    28
    24    25    25    30    29
    25    21    21    26    30

    31    32    33    34    35
    36    37    38    39    40
    31    32    32    37    36
    32    33    33    38    37
    33    34    34    39    38
    34    35    35    40    39
    35    31    31    36    40
    ];

C2 = patch('Vertices',C2_org,'Faces',C2_Faces,'FaceColor',"#535454");
%%%%%%%%%% C2 %%%%%%%%%%

%%%%%%%%%% C3 %%%%%%%%%%
t = 0.05:0.5:2*pi;
x1 = 30*cos(t);
y1 = 30*sin(t);
C3_1 = [x1' y1' zeros(size(x1))'];
C3_2 = C3_1;
C3_2(:,3) = 50;
C3_org = [C3_1;C3_2

-24 0 0
0 24 0
24 0 0
0 -24 0
-24 0 50
0 24 50
24 0 50
0 -24 50];

h_C3_org = [C3_org'; ones(1,size(C3_org,1))];
h_C3_org_ori = h_C3_org;
h_C3_org_ori = rotx(90)*h_C3_org_ori;
h_C3_org_ori = trans(0,25,18)*h_C3_org_ori;
random_pose = rand(1) * 300;
h_C3_org = trans(300+random_pose,230,HT)*h_C3_org;
C3_org = h_C3_org(1:3,:)';

C3_Faces = [1 2 3 4 5 6 7 8 9 10 11 12 13
    14 15 16 17 18 19 20 21 22 23 24 25 26
    1 2 15 15 15 15 15 15 15 15 15 15 14
    2 3 16 16 16 16 16 16 16 16 16 16 15
    3 4 17 17 17 17 17 17 17 17 17 17 16
    4 5 18 18 18 18 18 18 18 18 18 18 17
    5 6 19 19 19 19 19 19 19 19 19 19 18
    6 7 20 20 20 20 20 20 20 20 20 20 19
    7 8 21 21 21 21 21 21 21 21 21 21 20
    8 9 22 22 22 22 22 22 22 22 22 22 21
    9 10 23 23 23 23 23 23 23 23 23 23 22
    10 11 24 24 24 24 24 24 24 24 24 24 23
    11 12 25 25 25 25 25 25 25 25 25 25 24
    12 13 26 26 26 26 26 26 26 26 26 26 25
    13 1 1 1 1 1 1 1 1 1 1 14 26

    27 28 28 28 28 28 28 28 28 28 28 29 30
    31 32 33 33 33 33 33 33 33 33 33 33 34
    27 31 32 32 32 32 32 32 32 32 32 32 28
    28 32 33 33 33 33 33 33 33 33 33 33 29
    29 33 34 34 34 34 34 34 34 34 34 34 30
    30 34 27 27 27 27 27 27 27 27 27 27 27
    ];

C3 = patch('Vertices',C3_org,'Faces',C3_Faces,'FaceColor',"#535454");
%%%%%%%%%% C3 %%%%%%%%%%

if debug_mode == 0
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
    Qn = Qn-dQ;
    steps = 100;
    expected_pose = [0 900 0];
    h_C2_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C2_org;
    red = 1;
    pose_x = h_C2_exp(1,19)-robot_pose(1);
    pose_y = h_C2_exp(2,19)-robot_pose(2)+100+(DT2-370);
    pose_z = HT+25-robot_pose(3);
    phi_z = 1;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    QQ = [Qn'
        -j1 j2 90 -j4 100 j6
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    Q0 = QQ(:,1);
    Qf = QQ(:,3);
    t0 = 0;
    tf = 2;
    t = [0 1 2];
    %     [QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
    [QQ,~] = MultiPolyTrajV(QQ,steps,t,1);
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:size(QQ,2)
        MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
        [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
        objs=cat(3,C1,C2,C3);
        h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
        pos=[(-(((905+125+DL-160)-1030+50)-DA))/size(QQ,2)  0 0 0 0 0;0 900/size(QQ,2) 0 0 0 0;0 0 0 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C1_org = objs_final_pos(:,:,1);
        h_C2_org = objs_final_pos(:,:,2);
        h_C3_org = objs_final_pos(:,:,3);
    end

    if debug_mode == 0
        for n = 120:-1:100
            view(n,20)
            pause(0.01)
        end
    end

    % grasp the part with jacobian (GRASPING)
    if debug_mode == 0
        pause(0.3)
    end
    jacob_moves = [0 -150-(DT2-405) 0];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    Qn = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)]';
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        ]';
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
        Qn = Qn+dQ;
    end

    % pull part from the table IN Z AXIS
    if debug_mode == 0
        pause(0.3)
    end
    Qn = Qn-dQ;
    q = Qn';
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    jacob_moves = [0 0 200];
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[0 0 200/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % Rotate to main part
    x = x1;
    y = y1;
    z = z1;
    pose_x = h_C1_org(1,15)-robot_pose(1);
    pose_y = h_C1_org(2,15)-robot_pose(2)-100;
    pose_z = h_C1_org(3,15)-robot_pose(3)+50;
    phi_z = 91;
    theta_y = 1;
    psi_x = 91;
    M_rot = Trot(phi_z,theta_y,psi_x);
    TT = M_rot;
    TT(1,4) = pose_x;
    TT(2,4) = pose_y;
    TT(3,4) = pose_z;
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x1 = [TT(1,1:3) TT(1,4)];
    y1 = [TT(2,1:3) TT(2,4)];
    z1 = [TT(3,1:3) TT(3,4)];
    Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
    Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
    QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
        Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) Q_inv1(6,red)
        ]';
    dQ = (QQ(:,2) - QQ(:,1))/steps;
    Qn = QQ(:,1);
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        part_pose = AnimateRobot(H,AAA,P,h,0.01,0,robot_pose,1,h_C2_org_ori,C2);
        Qn = Qn+dQ;
    end
    h_C2_org = part_pose;

    if debug_mode == 0
        for n = 100:140
            view(n,n-80)
            pause(0.01)
        end
    end

    % move to main part X,Y AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [h_C1_org(1,15)-h_C2_org(1,23) h_C1_org(2,15)-h_C2_org(2,23) 0];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % move to main part Z AXIS
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [0 0 -(h_C2_org(3,19)-h_C1_org(3,5))];
    steps = 100;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        if n == 20
            view(90,90) % x,y
        end
        if n == 80
            view(120,20)
        end
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        objs=cat(3,C2);
        h_objs=cat(3,h_C2_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C2_org = objs_final_pos(:,:,1);
        Qn = Qn+dQ;
    end

    % drop part and move to second part
    if debug_mode == 0
        pause(0.3)
    end
    Qn = Qn-dQ;
    q = Qn';
    jacob_moves = [-300 0 0];
    steps = 50;
    TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
    for i=1:3
        if abs(TT(i,4)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    x = [TT(1,1:3) TT(1,4)];
    y = [TT(2,1:3) TT(2,4)];
    z = [TT(3,1:3) TT(3,4)];
    x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
    y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
    z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
    for i=1:3
        if abs(TT(i,4)+jacob_moves(i)) > max_range
            disp('Derired position is out of the range of the arm!')
            return
        end
    end
    dx = (x1(4) - x(4))/steps;
    dy = (y1(4) - y(4))/steps;
    dz = (z1(4) - z(4))/steps;
    dr = [dx;dy;dz;0;0;0];
    QQ = [Qn Qn];
    delete(h)
    delete(H)
    [H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
    for n=1:steps
        dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
        dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
        MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
        AAA = CalculateRobotMotion(MDH);
        AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
        Qn = Qn+dQ;
    end
    %%%%%%%%%% ROBOT - pick 1 %%%%%%%%%%
    view(220,20)
end

%%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%
steps = 100;
red = 1;
Qn = Qn-dQ;
q = Qn';
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
Q_inv = invkinUR10(x,y,z,LA,LC,LE,LF,LG,LH);
expected_pose = [0 960 0];
h_C3_exp = trans(expected_pose(1),expected_pose(2),expected_pose(3))*h_C3_org;
pose_x = h_C3_exp(1,33)-robot_pose(1)+60;
pose_y = h_C3_exp(2,33)-robot_pose(2);
pose_z = HT+25-robot_pose(3);
phi_z = 1;
theta_y = -91;
psi_x = 1;
M_rot = Trot(phi_z,theta_y,psi_x);
TT = M_rot;
TT(1,4) = pose_x;
TT(2,4) = pose_y;
TT(3,4) = pose_z;
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x1 = [TT(1,1:3) TT(1,4)];
y1 = [TT(2,1:3) TT(2,4)];
z1 = [TT(3,1:3) TT(3,4)];
Q_inv1 = invkinUR10(x1,y1,z1,LA,LC,LE,LF,LG,LH);
red2 = 1;
QQ = [Q_inv(1,red) Q_inv(2,red) Q_inv(3,red) Q_inv(4,red) Q_inv(5,red) Q_inv(6,red)
    Q_inv1(1,red2) Q_inv1(2,red2) Q_inv1(3,red2) Q_inv1(4,red2) Q_inv1(5,red2) 0
    ]';
Q0 = QQ(:,1);
Qf = QQ(:,2);
t0 = 0;
tf = 2;
[QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    [h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
    [h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);
    objs=cat(3,C1,C2,C3);
    h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
    pos=[0 0 0 0 0 0;0 0 0 0 0 0;0 960/steps 0 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C1_org = objs_final_pos(:,:,1);
    h_C2_org = objs_final_pos(:,:,2);
    h_C3_org = objs_final_pos(:,:,3);
end

% approach part 2
if debug_mode == 0
    pause(0.3)
end
q = [Q_inv1(1,red) Q_inv1(2,red) Q_inv1(3,red) Q_inv1(4,red) Q_inv1(5,red) 0];
Qn = q';
jacob_moves = [-80 0 0];
steps = 50;
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
    Qn = Qn+dQ;
end

% move part2 apart from small T
if debug_mode == 0
    pause(0.3)
end
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 0 250];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    objs=cat(3,C3);
    h_objs=cat(3,h_C3_org);
    pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C3_org = objs_final_pos(:,:,1);
    Qn = Qn+dQ;
end

% Rotate to main part
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if n>=2
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
    end
    objs=cat(3,C1,C2);
    h_objs=cat(3,h_C1_org,h_C2_org);
    pos=[-(DA+DB)/steps 0 0 0 0 0;-(DA+DB)/steps 0 0 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C1_org = objs_final_pos(:,:,1);
    h_C2_org = objs_final_pos(:,:,2);
    Qn = Qn+dQ;
end

% move to main part X,Y AXIS
Qn = Qn-dQ;
q = Qn';
jacob_moves = [(h_C1_org(1,15)-robot_pose(1))-(h_C3_org(1,32)-robot_pose(1)) (h_C1_org(2,15)-robot_pose(2))-(h_C3_org(2,32)-robot_pose(2)) 0];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    if n>=2
        objs=cat(3,C3);
        h_objs=cat(3,h_C3_org);
        pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
        N = 1;
        objs_final_pos = Move_Simult(objs,h_objs,N,pos);
        % update positions
        h_C3_org = objs_final_pos(:,:,1);
    end
    Qn = Qn+dQ;
end

if debug_mode == 0
    for n = 220:-1:150
        view(n,20)
        pause(0.01)
    end
end

% move to main part Z AXIS
Qn = Qn-dQ;
q = Qn';
jacob_moves = [0 0 -(h_C3_org(3,10)-h_C2_org(3,1))];
steps = 100;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    if n == 20
        view(90,90) % x,y
    end
    if n == 80
        view(120,20)
    end
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    objs=cat(3,C3);
    h_objs=cat(3,h_C3_org);
    pos=[jacob_moves(1)/steps jacob_moves(2)/steps jacob_moves(3)/steps 0 0 0];
    N = 1;
    objs_final_pos = Move_Simult(objs,h_objs,N,pos);
    % update positions
    h_C3_org = objs_final_pos(:,:,1);
    Qn = Qn+dQ;
end

% drop part and move
if debug_mode == 0
    pause(0.3)
end
Qn = Qn-dQ;
q = Qn';
jacob_moves = [200 0 0];
steps = 50;
TT = Multi_Trans(q,LA,LC,LE,LF,LG,LH);
for i=1:3
    if abs(TT(i,4)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
x = [TT(1,1:3) TT(1,4)];
y = [TT(2,1:3) TT(2,4)];
z = [TT(3,1:3) TT(3,4)];
x1 = [TT(1,1:3) TT(1,4)+jacob_moves(1)];
y1 = [TT(2,1:3) TT(2,4)+jacob_moves(2)];
z1 = [TT(3,1:3) TT(3,4)+jacob_moves(3)];
for i=1:3
    if abs(TT(i,4)+jacob_moves(i)) > max_range
        disp('Derired position is out of the range of the arm!')
        return
    end
end
dx = (x1(4) - x(4))/steps;
dy = (y1(4) - y(4))/steps;
dz = (z1(4) - z(4))/steps;
dr = [dx;dy;dz;0;0;0];
QQ = [Qn Qn];
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    dQ = jacobianUR10inv(Qn,LC,LE,LF,LG,LH)*dr;
    dQ = [dQ(1) dQ(2) dQ(3) dQ(4) dQ(5) dQ(6)]';
    MDH = GenerateMultiDH(DH_ur10e,Qn,jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.000001,0,robot_pose);
    Qn = Qn+dQ;
end
%%%%%%%%%% ROBOT - pick 2 %%%%%%%%%%

%%%%%%%%%% MOVE PARTS 3 %%%%%%%%%%
[h_C2_org,h_C3_org] = Norm_Matrix(h_C2_org,h_C3_org);
[h_C1_org,h_C2_org] = Norm_Matrix(h_C1_org,h_C2_org);

objs=cat(3,C1,C2,C3);
h_objs=cat(3,h_C1_org,h_C2_org,h_C3_org);
pos=[-890 0 0 0 0 0;-890 0 0 0 0 0;-890 0 0 0 0 0];
steps = 50;
objs_final_pos = Move_Simult(objs,h_objs,steps,pos);
% update positions
h_C1_org = objs_final_pos(:,:,1);
h_C2_org = objs_final_pos(:,:,2);
h_C3_org = objs_final_pos(:,:,3);
%%%%%%%%%% MOVE PARTS 3 %%%%%%%%%%

%%%%%%%%%% ROBOT - return to pause %%%%%%%%%%
Qn = Qn-dQ;
QQ = [Qn'
    j1 j2 j3 j4 j5 j6
    ]';
Q0 = QQ(:,1);
Qf = QQ(:,2);
t0 = 0;
tf = 2;
[QQ,~] = PolyTrajV(Q0,Qf,0,0,steps,t0,tf);
delete(h)
delete(H)
[H,h,P,~] = InitRobot(QQ,steps,DH_ur10e,jTypes,sScale,robot_pose);
for n=1:steps
    MDH = GenerateMultiDH(DH_ur10e, QQ(:,n), jTypes);
    AAA = CalculateRobotMotion(MDH);
    AnimateRobot(H,AAA,P,h,0.01,0,robot_pose);
end

%%%%%%%%%% ROBOT - return to pause %%%%%%%%%%

%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%