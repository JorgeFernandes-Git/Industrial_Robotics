function P = AnimateRobot(H,AAA,P,h,sd,draw,pose,grab,part,part_V)

% H - handles graficos dos objetos a animar (desenhados por DrawFrames)
%
% AAA - superhipermatriz que contem a sequencia temporal de
% hipermatrizes calculadas por CalculateRobotMotion;
%
% P - objeto a representar (sistema coordenadas: seixos3). É passado
% aqui para se recalcular dinamicamente a sua postura para fazer a
% representacao;
%
% h - handle grafico dos elos do robo (vem de DrawLinks);
%
% sd - indicador do nivel de pausa entre passos da animacao (controla a
% velocidade da animacao).

if ~exist('draw','var')
    draw = 1;
end

if ~exist('pose','var')
    pose = [0 0 0];
end

if ~exist('grab','var')
    grab = 0;
end

if ~exist('part','var')
    part = 0;
end

if ~exist('part_V','var')
    part_V = 0;
end

for n=1:size(AAA,4)
    org = LinkOrigins(AAA(:,:,:,n), pose(1), pose(2), pose(3));
    h.XData=org(1,:);
    h.YData=org(2,:);
    h.ZData=org(3,:);

    T = eye(4);
    for j=1:size(AAA,3)
        T = T*AAA(:,:,j,n); % pós multiplicar

        if j == size(AAA,3)
            [G,~] = gripper(1,50,80,20);
            G = rotx(90)*G;
            G = trans(0,10,0)*G;
            Pk = T*G;
        else
            Pk=T*P;
        end

        if grab == 1 && j == size(AAA,3)
            T_part = T;
%             T_part(1:3,3) = 1;
            partk = T_part * part;
            partk(1,:) = partk(1,:) + pose(1);
            partk(2,:) = partk(2,:) + pose(2);
            partk(3,:) = partk(3,:) + pose(3);
            part_V.Vertices = partk(1:3,:)';
            P = partk;
        end

        Pk(1,:) = Pk(1,:) + pose(1);
        Pk(2,:) = Pk(2,:) + pose(2);
        Pk(3,:) = Pk(3,:) + pose(3);

        H(j+1).Vertices = Pk(1:3,:)';

    end
    if draw == 1
        plot3(org(1,end), org(2,end), org(3,end),'.b','MarkerSize',5)
    end

    pause(sd)
end