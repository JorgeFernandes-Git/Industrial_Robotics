function [H,h,P,AAA] = InitRobot(QQ,NN,DH,jTypes,sScale, pose)


% Argumentos de entrada:
% QQ - matriz de colunas das posicoes de junta (pelo menos duas)
% NN - nÚmero de pontos de cada segmento do movimento
% DH - Matriz-base dos parâmetros cinemáticos
% jTypes - vetor com o tipo de juntas (0=rot, 1=prism; opcional)
% sScale - Fator de escala dos seixos3() (opcional: 1 por defeito)

% Valores de retorno:
% H - Handles gracos dos sistemas de eixos
% h - Handle graco dos elos do rob^o
% P - Objeto do seixos3() na escala usada
% AAA - Superhipermatriz com as transformac~oes das posic~oes do rob^

if ~exist('sScale','var')
    [P,F] = seixos3(0.3);
else
    [P,F] = seixos3(sScale);
end

if ~exist('jTypes','var')
    jTypes = [0 0 0 0 0 0 0 0 0];
end

if ~exist('pose','var')
    pose = [0 0 0];
end

MQ = [];
for k=1:size(QQ,2)-1
    MQ = [MQ, LinspaceVect(QQ(:,k),QQ(:,k+1),NN)];
end

% MQ = LinspaceVect(QQ(:,1),QQ(:,2),NN);
MDH = GenerateMultiDH(DH,MQ,jTypes);
AAA = CalculateRobotMotion(MDH);

colors=['r','g','b','r','g','b','r','g','b','r','g','b'];

Org = LinkOrigins(AAA(:,:,:,1), pose(1), pose(2), pose(3));
h = DrawLinks(Org);
H = DrawFrames(AAA(:,:,:,1),P,F,colors, pose(1), pose(2), pose(3));

