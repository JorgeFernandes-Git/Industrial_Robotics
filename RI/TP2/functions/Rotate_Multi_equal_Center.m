function objs_final_pos = Rotate_Multi_equal_Center(objs, h_objs, steps, center, angle)

% objs - multi dim array objs = cat(3, obj_1, obj_2, etc...)
% h_objs - multi dim array handles = cat(n_objs, obj_1, obj_2, etc...)
% steps
% center - center of rotation
% angle - angle of rotation

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

        objs(:,:,n).Vertices = n_objs_t;
        
        if i==1
            objs_final_pos(:,:,n) = n_objs;
        end
    end
    pause(0.05)
end