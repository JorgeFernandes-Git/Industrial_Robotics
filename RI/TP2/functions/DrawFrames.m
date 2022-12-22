function H = DrawFrames(AA,P,F,color,x,y,z)

if exist('x', 'var') && exist('y', 'var') && exist('z', 'var')
    Pi = P;
    Pi(1,:) = Pi(1,:) + x;
    Pi(2,:) = Pi(2,:) + y;
    Pi(3,:) = Pi(3,:) + z;
    H(1) = patch('Vertices', Pi(1:3,:)','Faces', F, 'FaceColor', 'w');
else
    P(1:3,:) = P(1:3,:);
    H(1) = patch('Vertices', P(1:3,:)','Faces', F, 'FaceColor', 'w');
end

% H=zeros(AA(3));

T = eye(4);

if ~exist('color','var')
    color = ['r','g','b'];
end

for k=1:size(AA,3)
    T = T*AA(:,:,k); % pós multiplicar

    if k == size(AA,3)
        [G,F] = gripper(1,50,80,20);
        G = rotx(90)*G;
        G = trans(0,10,0)*G;
        %         Pf = P*2;
        %         Pf(4,:) = 1;
        %         Pk = T*Pf;
        Pk = T*G;
    else
        Pk=T*P;
    end

    if exist('x', 'var') && exist('y', 'var') && exist('z', 'var')
        Pk(1,:) = Pk(1,:) + x;
        Pk(2,:) = Pk(2,:) + y;
        Pk(3,:) = Pk(3,:) + z;
    else
        Pk(1:3,:) = Pk(1:3,:);
    end
    
    H(k+1) = patch('Vertices', Pk(1:3,:)','Faces', F, 'FaceColor', color(k));
end

