function [TP1] = TP1_104580(theta, phi)

% FUNTIONS USED ARE IN THE END OF THE FILE

% close all;
% clc;
% clear;

if theta <= 0
    theta = theta + 90;
else
    theta = 90 - theta;
end

if phi <= 0
    phi = phi + 90;
else
    phi = 90 - phi;
end
% =======================================

debug_mode = 0;

% =======================================
% create workspace

fh = figure();
fh.WindowState = 'maximized';
hold on

grid on
view(135,30)
axis_dim = 50;
axis([0 axis_dim 0 axis_dim 0 40]) % define espaço de trabalho
axis square % evita a compressão dos eixos
% axis off
xlabel('x')
ylabel('y')
zlabel('z')
% =======================================

if debug_mode == 0
    blinks = 6;
else
    blinks = 1;
end

% prompt = {'Enter a value of \phi (in degrees)'};
% dlgtitle = 'phi Value';
% dims = [1 30];
% definput = {'0'};
% opts.Interpreter = 'tex';
% answer = inputdlg(prompt,dlgtitle,dims,definput,opts);
% angle_x = str2double(answer);
angle_x = 0;

% if isnan(angle_x)
%     angle_x = 45;
% end

% prompt = {'Enter a value of \theta (in degrees)'};
% dlgtitle = 'theta Value';
% dims = [1 30];
% definput = {'0'};
% opts.Interpreter = 'tex';
% answer = inputdlg(prompt,dlgtitle,dims,definput,opts);
% angle_z = str2double(answer);
angle_z = 0;

% if isnan(angle_z)
%     angle_z = 45;
% end

face_plate_dim = 30;
center_disp = face_plate_dim/2-10; % 10 is the intinial position

height_base=0.25;
thickness = height_base+2; % 2 is the real thickness

steps = 20;
small_pause = 0.8;
blink_pause = 0.2;
long_pause = 1;

colors = ["#A2142F" "#4DBEEE" "#77AC30" "#7E2F8E" "#EDB120" "#D95319" "#0072BD"];

% =======================================
% FACE PLATE
Face_Plate_2D = [0 0 height_base
    0 face_plate_dim height_base
    face_plate_dim face_plate_dim height_base
    face_plate_dim 0 height_base
    0 0 0
    0 face_plate_dim 0
    face_plate_dim face_plate_dim 0
    face_plate_dim 0 0];
Face_Plate_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_FACE_PLATE_org = [Face_Plate_2D';ones(1,size(Face_Plate_2D,1))];
Face_Plate_2D = Rotate3D_theta_phi(Face_Plate_2D, face_plate_dim, face_plate_dim, angle_x, angle_z);
FACE_PLATE = patch('Vertices',Face_Plate_2D,'Faces',Face_Plate_Faces,'FaceColor',"#f5f5f5");
% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% 1º big triangle
Triangle_Big_2D_org = [5+center_disp 15+center_disp thickness
    10+center_disp 15+center_disp thickness
    15+center_disp 15+center_disp thickness
    10+center_disp 10+center_disp thickness
    5+center_disp 15+center_disp height_base
    10+center_disp 15+center_disp height_base
    15+center_disp 15+center_disp height_base
    10+center_disp 10+center_disp height_base];

Triangle_Big_2D_org = Move_Init_pose(Triangle_Big_2D_org, [0 9 0 0 0 0]);
Triangle_Big_2D_org = Rotate_Init_pose(Triangle_Big_2D_org, [0 0 180]);

Triangle_Big_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_TRIANGLE_BIG_org = [Triangle_Big_2D_org';ones(1,size(Triangle_Big_2D_org,1))];
Triangle_Big_2D_rot = Rotate3D_theta_phi(Triangle_Big_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_TRIANGLE_BIG_rot = [Triangle_Big_2D_rot';ones(1,size(Triangle_Big_2D_rot,1))];
TRIANGLE_BIG = patch('Vertices',Triangle_Big_2D_rot,'Faces',Triangle_Big_Faces,'FaceColor',colors(1));
% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% 2º big triangle
Triangle_Big_2_2D_org = [15+center_disp 15+center_disp thickness
    15+center_disp 10+center_disp thickness
    15+center_disp 5+center_disp thickness
    10+center_disp 10+center_disp thickness
    15+center_disp 15+center_disp height_base
    15+center_disp 10+center_disp height_base
    15+center_disp 5+center_disp height_base
    10+center_disp 10+center_disp height_base];

Triangle_Big_2_2D_org = Move_Init_pose(Triangle_Big_2_2D_org, [8 4 0 0 0 0]);
Triangle_Big_2_2D_org = Rotate_Init_pose(Triangle_Big_2_2D_org,  [0 0 -150]);

Triangle_Big_2_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_TRIANGLE_BIG_2_org = [Triangle_Big_2_2D_org';ones(1,size(Triangle_Big_2_2D_org,1))];
Triangle_Big_2_2D_rot = Rotate3D_theta_phi(Triangle_Big_2_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_TRIANGLE_BIG_2_rot = [Triangle_Big_2_2D_rot';ones(1,size(Triangle_Big_2_2D_rot,1))];
TRIANGLE_BIG_2 = patch('Vertices',Triangle_Big_2_2D_rot,'Faces',Triangle_Big_2_Faces,'FaceColor',colors(2));
% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% square
Square_2D_org = [10+center_disp 10+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp thickness
    10+center_disp 5+center_disp thickness
    12.5+center_disp -5+10+sqrt(3.5^2 - 2.5^2)+center_disp thickness
    10+center_disp 10+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp height_base
    10+center_disp 5+center_disp height_base
    12.5+center_disp -5+10+sqrt(3.5^2 - 2.5^2)+center_disp height_base];

Square_2D_org = Move_Init_pose(Square_2D_org, [8 -4 0 0 0 0]);
Square_2D_org = Rotate_Init_pose(Square_2D_org,  [0 0 45]);

Square_2D_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_SQUARE_org = [Square_2D_org';ones(1,size(Square_2D_org,1))];
Square_2D_rot = Rotate3D_theta_phi(Square_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_SQUARE_rot = [Square_2D_rot';ones(1,size(Square_2D_rot,1))];
SQUARE = patch('Vertices',Square_2D_rot,'Faces',Square_2D_Faces,'FaceColor',colors(7));
% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% small triangle bottom
Triangle_Small_2D_org = [10+center_disp 5+center_disp thickness
    12.5+center_disp 5+center_disp thickness
    15+center_disp 5+center_disp thickness
    12.5+center_disp -5+10+sqrt(3.5^2 - 2.5^2)+center_disp thickness
    10+center_disp 5+center_disp height_base
    12.5+center_disp 5+center_disp height_base
    15+center_disp 5+center_disp height_base
    12.5+center_disp -5+10+sqrt(3.5^2 - 2.5^2)+center_disp height_base];

Triangle_Small_2D_org = Move_Init_pose(Triangle_Small_2D_org, [-3 -7 0 0 0 0]);
Triangle_Small_2D_org = Rotate_Init_pose(Triangle_Small_2D_org,  [0 0 180]);

Triangle_Small_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_TRIANGLE_SMALL_org = [Triangle_Small_2D_org';ones(1,size(Triangle_Small_2D_org,1))];
Triangle_Small_2D_rot = Rotate3D_theta_phi(Triangle_Small_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_TRIANGLE_SMALL_rot = [Triangle_Small_2D_rot';ones(1,size(Triangle_Small_2D_rot,1))];
TRIANGLE_SMALL = patch('Vertices',Triangle_Small_2D_rot,'Faces',Triangle_Small_Faces,'FaceColor',colors(3));
% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% medium triangle
Triangle_Medium_2D_org = [10+center_disp 5+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp thickness
    5+center_disp 10+center_disp thickness
    5+center_disp 5+center_disp thickness
    10+center_disp 5+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp height_base
    5+center_disp 10+center_disp height_base
    5+center_disp 5+center_disp height_base];

Triangle_Medium_2D_org = Move_Init_pose(Triangle_Medium_2D_org, [-6 -6 0 0 0 0]);
Triangle_Medium_2D_org = Rotate_Init_pose(Triangle_Medium_2D_org,  [0 0 180]);

Triangle_Medium_2D_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_TRIANGLE_MEDIUM_org = [Triangle_Medium_2D_org';ones(1,size(Triangle_Medium_2D_org,1))];
Triangle_Medium_2D_rot = Rotate3D_theta_phi(Triangle_Medium_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_TRIANGLE_MEDIUM_rot = [Triangle_Medium_2D_rot';ones(1,size(Triangle_Medium_2D_rot,1))];
TRIANGLE_MEDIUM = patch('Vertices',Triangle_Medium_2D_rot,'Faces',Triangle_Medium_2D_Faces,'FaceColor',colors(5));

% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% small triangle center
Triangle_Small_center_2D_org = [10+center_disp 10+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 10+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 12.5+center_disp thickness
    10+center_disp 10+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 10+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 12.5+center_disp height_base];

Triangle_Small_center_2D_org = Move_Init_pose(Triangle_Small_center_2D_org, [-10 -1 0 0 0 0]);
Triangle_Small_center_2D_org = Rotate_Init_pose(Triangle_Small_center_2D_org,  [0 0 180]);

Triangle_Small_center_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_TRIANGLE_SMALL_CENTER_org = [Triangle_Small_center_2D_org';ones(1,size(Triangle_Small_center_2D_org,1))];
Triangle_Small_center_2D_rot = Rotate3D_theta_phi(Triangle_Small_center_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_TRIANGLE_SMALL_CENTER_rot = [Triangle_Small_center_2D_rot';ones(1,size(Triangle_Small_center_2D_rot,1))];
TRIANGLE_SMALL_CENTER = patch('Vertices',Triangle_Small_center_2D_rot,'Faces',Triangle_Small_center_Faces,'FaceColor',colors(4));
% =======================================
if debug_mode == 0
    pause(small_pause)
end
% =======================================
% trapeze
Trapeze_2D_org = [5+center_disp 15+center_disp thickness
    5+center_disp 10+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp thickness
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 12.5+center_disp thickness
    5+center_disp 15+center_disp height_base
    5+center_disp 10+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 7.5+center_disp height_base
    -5+10+sqrt(3.5^2 - 2.5^2)+center_disp 12.5+center_disp height_base];

Trapeze_2D_org = Move_Init_pose(Trapeze_2D_org, [-5 6 0 0 0 0]);
Trapeze_2D_org = Rotate_Init_pose(Trapeze_2D_org,  [0 0 -40]);

Trapeze_2D_Faces = [1 2 3 4
    5 6 7 8
    1 2 6 5
    2 3 7 6
    3 4 8 7
    4 1 5 8];
h_TRAPEZE_org = [Trapeze_2D_org';ones(1,size(Trapeze_2D_org,1))];
Trapeze_2D_rot = Rotate3D_theta_phi(Trapeze_2D_org, face_plate_dim, face_plate_dim, angle_x, angle_z);
h_TRAPEZE_rot = [Trapeze_2D_rot';ones(1,size(Trapeze_2D_rot,1))];
TRAPEZE = patch('Vertices',Trapeze_2D_rot,'Faces',Trapeze_2D_Faces,'FaceColor',colors(6));
% =======================================
% positioning the plate and the objects
objs=cat(3,FACE_PLATE,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_FACE_PLATE_org,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
pos=[0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0; 0 0 face_plate_dim*sind(theta) 0 0 0];
steps = 20;

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x, angle_z);

angle_x = theta;
angle_z = phi;
objs_final_pos = Rotate_Moving_theta_phi_Simult(objs,h_objs,steps,face_plate_dim, face_plate_dim, angle_x, 0);

% update positions
h_FACE_PLATE_rot = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_rot = objs_final_pos(:,:,2);
h_TRIANGLE_BIG_2_rot = objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_rot= objs_final_pos(:,:,4);
h_TRIANGLE_SMALL_CENTER_rot= objs_final_pos(:,:,5);
h_TRIANGLE_MEDIUM_rot = objs_final_pos(:,:,6);
h_TRAPEZE_rot = objs_final_pos(:,:,7);
h_SQUARE_rot = objs_final_pos(:,:,8);

objs=cat(3,FACE_PLATE,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_FACE_PLATE_rot,h_TRIANGLE_BIG_rot,h_TRIANGLE_BIG_2_rot,h_TRIANGLE_SMALL_rot,h_TRIANGLE_SMALL_CENTER_rot,h_TRIANGLE_MEDIUM_rot,h_TRAPEZE_rot,h_SQUARE_rot);
angle_x = 0;
angle_z = 0;
pos=[0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0; 0 face_plate_dim*sind(phi) 0 0 0 0];
objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x, angle_z);

angle_x = theta;
angle_z = phi;
objs_final_pos = Rotate_Moving_theta_phi_Simult(objs,h_objs,steps,face_plate_dim, face_plate_dim, 0, angle_z);

% =========================================================================
% START MOVES
for i=1:10
    axis_dim = 50-i;
    axis([0 axis_dim 0 axis_dim 0 40]) % define espaço de trabalho
    pause(0.05)
end
if debug_mode == 0
    pause(long_pause)
end
% =========================================================================
% rotate all 360º

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
eq_center = [face_plate_dim/2 face_plate_dim/2 0];
eq_angles = 360;
angles = [360 360 360 360 360 360 360];

if debug_mode == 1
    steps = 2;
else
    steps = 100;
end

objs_final_pos = Rotate_own_Center_and_Rotate_equal_Center_Multi(objs,h_objs,steps,eq_center,eq_angles, angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =======================================

% =======================================
% positioning to make square
% rotação

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
angles = [-180 150 -180 -180 -180 40 -45];

objs_final_pos = Rotate_Multi_obj_own_Center(objs,h_objs,steps,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 10 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; -8 4 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% final pose PUZZLE SQUARE
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[0 -9 0 0 0 0; -5.04 -2.30 0 0 0 0; 3 7 0 0 0 0; 0 0 0 0 0 0; 6 6 0 0 0 0; 5 -6 0 0 0 0; 0 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
center = [face_plate_dim/2 face_plate_dim/2 0];
angle = 180;
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end

objs_final_pos = Rotate_Multi_equal_Center(objs,h_objs,steps,center,angle,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% UPDATE FINAL COLOR -- PUZZLE SQUARE

for j = 1:blinks
    TRIANGLE_BIG.FaceColor = colors(3);
    TRIANGLE_BIG_2.FaceColor = colors(3);
    TRIANGLE_SMALL.FaceColor = colors(3);
    TRIANGLE_SMALL_CENTER.FaceColor = colors(3);
    TRIANGLE_MEDIUM.FaceColor = colors(3);
    TRAPEZE.FaceColor = colors(3);
    SQUARE.FaceColor = colors(3);
    pause(blink_pause)
    TRIANGLE_BIG.FaceColor = colors(4);
    TRIANGLE_BIG_2.FaceColor = colors(4);
    TRIANGLE_SMALL.FaceColor = colors(4);
    TRIANGLE_SMALL_CENTER.FaceColor = colors(4);
    TRIANGLE_MEDIUM.FaceColor = colors(4);
    TRAPEZE.FaceColor = colors(4);
    SQUARE.FaceColor = colors(4);
    pause(blink_pause)
end
TRIANGLE_BIG.FaceColor = colors(3);
TRIANGLE_BIG_2.FaceColor = colors(3);
TRIANGLE_SMALL.FaceColor = colors(3);
TRIANGLE_SMALL_CENTER.FaceColor = colors(3);
TRIANGLE_MEDIUM.FaceColor = colors(3);
TRAPEZE.FaceColor = colors(3);
SQUARE.FaceColor = colors(3);

if debug_mode == 0
    pause(long_pause)
end
% FIRST PUZZLE COMPLETE ==================================================
% ========================================================================

% restaure origial colors
TRIANGLE_BIG.FaceColor = colors(1);
TRIANGLE_BIG_2.FaceColor = colors(2);
TRIANGLE_SMALL.FaceColor = colors(3);
TRIANGLE_SMALL_CENTER.FaceColor = colors(4);
TRIANGLE_MEDIUM.FaceColor = colors(5);
TRAPEZE.FaceColor = colors(6);
SQUARE.FaceColor = colors(7);

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[0 -7 0 0 0 0; -7 -7 0 0 0 0; -7 7 0 0 0 0; 6 3 0 0 0 0; 7 7 0 0 0 0; 7 -7 0 0 0 0; 0 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================
% positioning to make PUZZLE CHIKEN
% rotação

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
angles = [0 -90 0 -90 -90 45 0];

objs_final_pos = Rotate_Multi_obj_own_Center(objs,h_objs,steps,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[1 7 0 0 0 0; 3 13 0 0 0 0; 0 0 0 0 0 0; -2 -1 0 0 0 0; 0 0 0 0 0 0; -2 5 0 0 0 0; 1 1 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 5 0 0 0 0 0; 0 0 0 0 0 0; -1.8 0.3 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[-5 6 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; -2.8 2.5 0 0 0 0; 0 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[3 -2 0 0 0 0; 3 -2 0 0 0 0; 0 -13 0 0 0 0; 3 -2 0 0 0 0; 0 0 0 0 0 0; 3 -2 0 0 0 0; 3 -2 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[-3 0 0 0 0 0; -3 0 0 0 0 0; 0 0 0 0 0 0; -3 0 0 0 0 0; -21 0 0 0 0 0; -3 0 0 0 0 0; -3 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[3 0 0 0 0 0; 3 0 0 0 0 0; 6 0 0 0 0 0; 3 0 0 0 0 0; 0 -16 0 0 0 0; 3 0 0 0 0 0; 3 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[-3 0 0 0 0 0; -3 0 0 0 0 0; -3 0 0 0 0 0; -3 0 0 0 0 0; 5 0 0 0 0 0; -3 0 0 0 0 0; -3 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 3 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% UPDATE FINAL COLOR -- PUZZLE KITCHEN

for j = 1:blinks
    TRIANGLE_BIG.FaceColor = colors(1);
    TRIANGLE_BIG_2.FaceColor = colors(1);
    TRIANGLE_SMALL.FaceColor = colors(1);
    TRIANGLE_SMALL_CENTER.FaceColor = colors(1);
    TRIANGLE_MEDIUM.FaceColor = colors(1);
    TRAPEZE.FaceColor = colors(1);
    SQUARE.FaceColor = colors(1);
    pause(blink_pause)
    TRIANGLE_BIG.FaceColor = colors(4);
    TRIANGLE_BIG_2.FaceColor = colors(4);
    TRIANGLE_SMALL.FaceColor = colors(4);
    TRIANGLE_SMALL_CENTER.FaceColor = colors(4);
    TRIANGLE_MEDIUM.FaceColor = colors(4);
    TRAPEZE.FaceColor = colors(4);
    SQUARE.FaceColor = colors(4);
    pause(blink_pause)
end
TRIANGLE_BIG.FaceColor = colors(1);
TRIANGLE_BIG_2.FaceColor = colors(1);
TRIANGLE_SMALL.FaceColor = colors(1);
TRIANGLE_SMALL_CENTER.FaceColor = colors(1);
TRIANGLE_MEDIUM.FaceColor = colors(1);
TRAPEZE.FaceColor = colors(1);
SQUARE.FaceColor = colors(1);

if debug_mode == 0
    pause(long_pause)
end
% SECOND PUZZLE COMPLETE ==================================================
% ========================================================================
% ========================================================================

% restaure origial colors
TRIANGLE_BIG.FaceColor = colors(1);
TRIANGLE_BIG_2.FaceColor = colors(2);
TRIANGLE_SMALL.FaceColor = colors(3);
TRIANGLE_SMALL_CENTER.FaceColor = colors(4);
TRIANGLE_MEDIUM.FaceColor = colors(5);
TRAPEZE.FaceColor = colors(6);
SQUARE.FaceColor = colors(7);


% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[0 0 0 0 0 0; 0 7 0 0 0 0; 0 0 0 0 0 0; 5 0 0 0 0 0; 3 -5 0 0 0 0; 2 -4 0 0 0 0; 0 0 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[-3 4 0 0 0 0; 0 0 0 0 0 0; -2 -2 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 5 5 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

% =========================================================================
% rotate all 360º

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
eq_center = [face_plate_dim/2 face_plate_dim/2 0];
eq_angles = 270;
angles = [360 360 360 360 360 360 360];

if debug_mode == 1
    steps = 2;
else
    steps = 100;
end

objs_final_pos = Rotate_own_Center_and_Rotate_equal_Center_Multi(objs,h_objs,steps,eq_center,eq_angles, angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =========================================================================
% =========================================================================
% positioning to make PUZZLE CHILD PLAYING
% rotação

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
angles = [135 0 -90 0 -180 45 0];

objs_final_pos = Rotate_Multi_obj_own_Center(objs,h_objs,steps,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[-5 0 0 0 0 0; -7.71 2.2 0 0 0 0; -2 -0.6 0 0 0 0; 0 0 0 0 0 0; 4.7 0.7 0 0 0 0; 0 0 0 0 0 0; 0 -2 0 0 0 0];

objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; -5 9 0 0 0 0; 0 0 0 0 0 0; 2 0 0 0 0 0; -7 7 0 0 0 0];
objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 -8 0 0 0 0; 0 0 0 0 0 0; 11.8 0 0 0 0 0; 0 0 0 0 0 0];
objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 11.8 0 0 0 0 0; 0 0 0 0 0 0; 0 13 0 0 0 0; 0 0 0 0 0 0];
objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 8.7 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 6.25 -4 0 0 0 0];
objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 1.16 0 0 0 0];
objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =======================================
% =======================================
objs=cat(3,SQUARE);
h_objs=cat(3,h_SQUARE_org);
center = [20.8 13.7 0];
angle = -20;
if debug_mode == 1
    steps = 2;
else
    steps = 50;
end

objs_final_pos = Rotate_Multi_equal_Center(objs,h_objs,steps,center,angle,face_plate_dim,face_plate_dim,angle_x,angle_z);
% update positions
h_SQUARE_org = objs_final_pos(:,:,1);

% UPDATE FINAL COLOR -- PUZZLE KITCHEN

for j = 1:blinks
    TRIANGLE_BIG.FaceColor = colors(2);
    TRIANGLE_BIG_2.FaceColor = colors(2);
    TRIANGLE_SMALL.FaceColor = colors(2);
    TRIANGLE_SMALL_CENTER.FaceColor = colors(2);
    TRIANGLE_MEDIUM.FaceColor = colors(2);
    TRAPEZE.FaceColor = colors(2);
    SQUARE.FaceColor = colors(2);
    pause(blink_pause)
    TRIANGLE_BIG.FaceColor = colors(4);
    TRIANGLE_BIG_2.FaceColor = colors(4);
    TRIANGLE_SMALL.FaceColor = colors(4);
    TRIANGLE_SMALL_CENTER.FaceColor = colors(4);
    TRIANGLE_MEDIUM.FaceColor = colors(4);
    TRAPEZE.FaceColor = colors(4);
    SQUARE.FaceColor = colors(4);
    pause(blink_pause)
end
TRIANGLE_BIG.FaceColor = colors(2);
TRIANGLE_BIG_2.FaceColor = colors(2);
TRIANGLE_SMALL.FaceColor = colors(2);
TRIANGLE_SMALL_CENTER.FaceColor = colors(2);
TRIANGLE_MEDIUM.FaceColor = colors(2);
TRAPEZE.FaceColor = colors(2);
SQUARE.FaceColor = colors(2);

if debug_mode == 0
    pause(long_pause)
end
% THIRD PUZZLE COMPLETE ==================================================
% ========================================================================
% ========================================================================
% ========================================================================
% ========================================================================
% restaure origial colors
TRIANGLE_BIG.FaceColor = colors(1);
TRIANGLE_BIG_2.FaceColor = colors(2);
TRIANGLE_SMALL.FaceColor = colors(3);
TRIANGLE_SMALL_CENTER.FaceColor = colors(4);
TRIANGLE_MEDIUM.FaceColor = colors(5);
TRAPEZE.FaceColor = colors(6);
SQUARE.FaceColor = colors(7);

% exiting with style!!!

% =======================================
objs=cat(3,SQUARE);
h_objs=cat(3,h_SQUARE_org);
center = [20.8 13.7 0];
angle = 40;
if debug_mode == 1
    steps = 2;
else
    steps = 5;
end

objs_final_pos = Rotate_Multi_equal_Center(objs,h_objs,steps,center,angle,face_plate_dim,face_plate_dim,angle_x,angle_z);
% update positions
h_SQUARE_org = objs_final_pos(:,:,1);

% =======================================
% =======================================
objs=cat(3,SQUARE);
h_objs=cat(3,h_SQUARE_org);
center = [20.8 13.7 0];
angle = -40;
if debug_mode == 1
    steps = 2;
else
    steps = 5;
end

objs_final_pos = Rotate_Multi_equal_Center(objs,h_objs,steps,center,angle,face_plate_dim,face_plate_dim,angle_x,angle_z);
% update positions
h_SQUARE_org = objs_final_pos(:,:,1);

% =======================================
% translação com rotação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 50;
end
pos=[0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; -30 -30 0];
angles = [0 0 0 0 0 0 -360];

objs_final_pos = Rotate_own_Center_and_Trans_Multi(objs,h_objs,steps,pos,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 50 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 50 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =======================================
% =======================================

% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 10;
end
pos=[0 50 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =======================================
% =========================================================================
% rotate all 360º

objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
center = [20.8 18.8 0];
angle = 90;

if debug_mode == 1
    steps = 2;
else
    steps = 10;
end

objs_final_pos = Rotate_Multi_equal_Center(objs,h_objs,steps,center,angle,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =========================================================================
% translação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 2;
else
    steps = 20;
end
pos=[0 0 0 0 0 0; 0 -50 0 0 0 0; 0 0 0 0 0 0; 50 0 0 0 0 0; 0 0 0 0 0 0; -50 0 0 0 0 0; 0 0 0 0 0 0];objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);
% =========================================================================
% =======================================
% translação com rotação
objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
if debug_mode == 1
    steps = 20;
else
    steps = 50;
end
pos=[35 9 0; -3 40 0; 40 -2 0; -50 0 0; 35 5 0; 45 5 0; -35 25 0];
angles = [-255 -255 -255 -255 -255 -255 -255];

objs_final_pos = Rotate_own_Center_and_Trans_Multi(objs,h_objs,steps,pos,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

% update positions
h_TRIANGLE_BIG_org = objs_final_pos(:,:,1);
h_TRIANGLE_BIG_2_org = objs_final_pos(:,:,2);
h_TRIANGLE_SMALL_org= objs_final_pos(:,:,3);
h_TRIANGLE_SMALL_CENTER_org= objs_final_pos(:,:,4);
h_TRIANGLE_MEDIUM_org = objs_final_pos(:,:,5);
h_TRAPEZE_org = objs_final_pos(:,:,6);
h_SQUARE_org = objs_final_pos(:,:,7);

end
% =======================================
%*********************** END **********************************************
% functions used
function M=trans(x,y,z)

M = [1 0 0 x
    0 1 0 y
    0 0 1 z
    0 0 0 1];
end
% =======================================
function M=rotx(a)


M=[1    0      0    0
    0 cosd(a) -sind(a) 0
    0 sind(a) cosd(a)  0
    0    0      0    1];
end
% =======================================
function M=roty(a)


M=[cosd(a)  0 sind(a) 0
    0       1   0    0
    -sind(a) 0 cosd(a) 0
    0       0   0    1];

end
% =======================================
function M=rotz(a)


M=[cosd(a) -sind(a) 0 0
    sind(a) cosd(a)  0 0
    0        0     1 0
    0        0     0 1];

end
% =======================================
function obj_final_pos = Move_Init_pose(obj,pos)

% obj - passed object
% pos - [x, y, z, alpha, phi, phi]

obj = [obj';ones(1,size(obj,1))];
obj = trans(pos(1),pos(2),pos(3))*rotx(pos(4))*roty(pos(5))*rotz(pos(6))*obj;
obj=obj';
obj_final_pos = obj(:,1:3);

end
% =======================================
function objs_final_pos = Move_Simult_3D(objs, h_objs, steps, pos, dim_x, dim_z, theta_x, phi_z)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% pos - [x, y, z, alpha, theta, phi]

% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% theta_x - angle of rotation in x
% phi_z - angle of rotation in z

% example:
% objs=cat(3,TRIANGLE_BIG);
% h_objs=cat(3,h_TRIANGLE_BIG_org);
% pos=[0 7.5 0 0 0 0];
% objs_final_pos = Move_Simult_3D(objs,h_objs,steps,pos,face_plate_dim,face_plate_dim,angle_x,angle_z);


for i=linspace(0, 1, steps)
    for n=1:size(objs,3)
        n_objs = trans(i*pos(n,1),i*pos(n,2),i*pos(n,3))*rotx(i*pos(n,4))*roty(i*pos(n,5))*rotz(i*pos(n,6))*h_objs(:,:,n);
        n_objs_t = n_objs';
        n_objs_t = n_objs_t(:,1:3);

        n_objs_rot = Rotate3D_theta_phi(n_objs_t,dim_x, dim_z, theta_x, phi_z);

        objs(:,:,n).Vertices = n_objs_rot;

        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end
end
% =======================================
function obj_final_pos=Rotate3D_theta_phi(obj, dim_x, dim_z, theta_x, phi_z)
% obj - passed object
% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% theta_x - angle of rotation in x
% phi_z - angle of rotation in z

% example:
% Face_Plate_2D = Rotate3D_theta_phi(Face_Plate_2D, face_plate_dim, face_plate_dim, angle_x, angle_z);

obj = [obj'; ones(1,size(obj,1))];
obj = trans(0,dim_z*sind(phi_z),0)*rotz(-phi_z)*trans(0,0,dim_x*sind(theta_x))*rotx(-theta_x)*obj;

obj=obj';
obj_final_pos = obj(:,1:3);
end
% =======================================
function obj_final_pos = Rotate_Init_pose(obj, angle)
% obj - passed object
% angle - angle of rotation

c1 = max(obj(:,1))-((max(obj(:,1))-min(obj(:,1)))/2);
c2 = max(obj(:,2))-((max(obj(:,2))-min(obj(:,2)))/2);

obj = [obj';ones(1,size(obj,1))];
obj = trans(c1,c2,0)*rotx(angle(1))*roty(angle(2))*rotz(angle(3))*trans(-c1,-c2,-0)*obj;

obj=obj';
obj_final_pos = obj(:,1:3);

end
% =======================================
function objs_final_pos=Rotate_Moving_theta_phi_Simult(objs, h_objs, steps, dim_x, dim_z, theta_x, phi_z)

for i=linspace(0, 1, steps)
    for n=1:size(objs,3)
        n_objs = trans(0,dim_z*sind(phi_z),0)*rotz(i*-phi_z)*trans(0,0,dim_x*sind(theta_x))*rotx(i*-theta_x)*h_objs(:,:,n);
        n_objs_t = n_objs';
        n_objs_t = n_objs_t(:,1:3);

        objs(:,:,n).Vertices = n_objs_t;

        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end

end
% =======================================
function objs_final_pos = Rotate_Multi_equal_Center(objs, h_objs, steps, center, angle, dim_x, dim_z, theta_x, phi_z)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% center - center of rotation
% angle - angle of rotation

% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% theta_x - angle of rotation in x
% phi_z - angle of rotation in z

% example:
% objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
% h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
% center = [face_plate_dim/2 face_plate_dim/2 0];
% angle = 360;
% steps = 100;
%
% objs_final_pos = Rotate_Multi_equal_Center(objs,h_objs,steps,center,angle,face_plate_dim,face_plate_dim,angle_x,angle_z);


for i=linspace(0, 1, steps)
    for n=1:size(objs,3)
        n_objs = trans(center(1),center(2),center(3))*rotz(i*angle)*trans(-center(1),-center(2),-center(3))*h_objs(:,:,n);
        n_objs_t = n_objs';
        n_objs_t = n_objs_t(:,1:3);

        n_objs_rot = Rotate3D_theta_phi(n_objs_t,dim_x, dim_z, theta_x, phi_z);

        objs(:,:,n).Vertices = n_objs_rot;

        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end

end
% =======================================
function objs_final_pos = Rotate_Multi_obj_own_Center(objs, h_objs, steps, angles, dim_x, dim_z, theta_x, phi_z)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% angles- angles of rotation for each obj

% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% theta_x - angle of rotation in x
% phi_z - angle of rotation in z

% example:
% objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
% h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
% steps = 50;
% angles = [360 360 360 360 360 360 360];
%
% objs_final_pos = Rotate_Multi_obj_own_Center(objs,h_objs,steps,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

for i=linspace(0, 1, steps)
    for n=1:size(objs,3)

        obj = h_objs(:,:,n)';

        c1 = max(obj(:,1))-((max(obj(:,1))-min(obj(:,1)))/2);
        c2 = max(obj(:,2))-((max(obj(:,2))-min(obj(:,2)))/2);

        n_objs = trans(c1,c2,0)*rotz(i*angles(n))*trans(-c1,-c2,-0)*h_objs(:,:,n);
        n_objs_t = n_objs';
        n_objs_t = n_objs_t(:,1:3);

        n_objs_rot = Rotate3D_theta_phi(n_objs_t,dim_x, dim_z, theta_x, phi_z);

        objs(:,:,n).Vertices = n_objs_rot;

        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end

end
% =======================================
function objs_final_pos = Rotate_own_Center_and_Rotate_equal_Center_Multi(objs, h_objs, steps, eq_center, eq_angles, angles, dim_x, dim_z, theta_x, phi_z)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% angles- angles of rotation for each obj
% eq_center - center of rotation all objs
% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% theta_x - angle of rotation in x
% phi_z - angle of rotation in z

% objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
% h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
% eq_center = [face_plate_dim/2 face_plate_dim/2 0];
% eq_angles = 360;
% angles = [360 360 360 360 360 360 360];
% steps = 100;
% objs_final_pos = Rotate_own_Center_and_Rotate_equal_Center_Multi(objs,h_objs,steps,eq_center,eq_angles, angles,face_plate_dim,face_plate_dim,angle_x,angle_z);


for i=linspace(0, 1, steps)
    for n=1:size(objs,3)

        obj = trans(eq_center(1),eq_center(2),eq_center(3))*rotz(i*eq_angles)*trans(-eq_center(1),-eq_center(2),-eq_center(3))*h_objs(:,:,n);
        obj = obj';

        c1 = max(obj(:,1))-((max(obj(:,1))-min(obj(:,1)))/2);
        c2 = max(obj(:,2))-((max(obj(:,2))-min(obj(:,2)))/2);

        n_objs = trans(c1,c2,0)*rotz(i*angles(n))*trans(-c1,-c2,-0)*obj';
        n_objs_t = n_objs';
        n_objs_t = n_objs_t(:,1:3);

        n_objs_rot = Rotate3D_theta_phi(n_objs_t,dim_x, dim_z, theta_x, phi_z);

        objs(:,:,n).Vertices = n_objs_rot;

        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end

end
% =======================================
function objs_final_pos = Rotate_own_Center_and_Trans_Multi(objs, h_objs, steps, pos, angles, dim_x, dim_z, theta_x, phi_z)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% angles- angles of rotation for each obj

% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% theta_x - angle of rotation in x
% phi_z - angle of rotation in z

% example:
% objs=cat(3,TRIANGLE_BIG,TRIANGLE_BIG_2,TRIANGLE_SMALL,TRIANGLE_SMALL_CENTER,TRIANGLE_MEDIUM,TRAPEZE,SQUARE);
% h_objs=cat(3,h_TRIANGLE_BIG_org,h_TRIANGLE_BIG_2_org,h_TRIANGLE_SMALL_org,h_TRIANGLE_SMALL_CENTER_org,h_TRIANGLE_MEDIUM_org,h_TRAPEZE_org,h_SQUARE_org);
% steps = 50;
% angles = [360 360 360 360 360 360 360];
%
% objs_final_pos = Rotate_Multi_obj_own_Center(objs,h_objs,steps,angles,face_plate_dim,face_plate_dim,angle_x,angle_z);

for i=linspace(0, 1, steps)
    for n=1:size(objs,3)

        obj = trans(i*pos(n,1),i*pos(n,2),i+pos(n,3))*h_objs(:,:,n);
        obj = obj';

        c1 = max(obj(:,1))-((max(obj(:,1))-min(obj(:,1)))/2);
        c2 = max(obj(:,2))-((max(obj(:,2))-min(obj(:,2)))/2);

        n_objs = trans(c1,c2,0)*rotz(i*angles(n))*trans(-c1,-c2,-0)*obj';
        n_objs_t = n_objs';
        n_objs_t = n_objs_t(:,1:3);

        n_objs_rot = Rotate3D_theta_phi(n_objs_t,dim_x, dim_z, theta_x, phi_z);

        objs(:,:,n).Vertices = n_objs_rot;

        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end

end
% =======================================



