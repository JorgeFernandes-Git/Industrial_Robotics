function Org=LinkOrigins(AA, x, y, z)

Org = zeros(4,size(AA,3)+1);
Org(4,:) = 1; % coordenada homogenea

T = eye(4);

for k=1:size(AA,3)
    T = T*AA(:,:,k); % pós multiplicar
    Org(1:3,k+1) = T(1:3,4);
end

if exist('x', 'var') && exist('y', 'var') && exist('z', 'var')
    Org(1,:) = Org(1,:) + x;
    Org(2,:) = Org(2,:) + y;
    Org(3,:) = Org(3,:) + z;
else
    Org(1:3,:) = Org(1:3,:);
end