function objs_final_pos = Move_Simult(objs, h_objs, steps, pos)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% pos - [x, y, z, alpha, phi, rho]

% dim_x - dimension of rotation in x
% dim_z - dimension of rotation in z
% phi_x - angle of rotation in x
% rho_z - angle of rotation in z

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

        objs(:,:,n).Vertices = n_objs_t;
        
        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end

