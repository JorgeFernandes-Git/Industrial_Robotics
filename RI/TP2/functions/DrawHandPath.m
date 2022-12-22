function P = DrawHandPath(AAA,pose,grab,part)

if ~exist('pose','var')
    pose = [0 0 0];
end

if ~exist('grab','var')
    grab = 0;
end

if ~exist('part','var')
    part = 0;
end

for n=1:size(AAA,4)
    org = LinkOrigins(AAA(:,:,:,n), pose(1), pose(2), pose(3));
    plot3(org(1,end), org(2,end), org(3,end),'.b','MarkerSize',5)

    if grab == 1
        T = eye(4);
        for j=1:size(AAA,3)
            T = T*AAA(:,:,j,n); % pós multiplicar

            if j == size(AAA,3)
                T_part = T;
                %             T_part(1:3,3) = 1;
                partk = T_part * part;
                partk(1,:) = partk(1,:) + pose(1);
                partk(2,:) = partk(2,:) + pose(2);
                partk(3,:) = partk(3,:) + pose(3);
                P = partk;
            end
        end
    end
    pause(0.001)
   
end

