clc
clear
close all
delete(imaqfind);
addpath Calibration_Images\
addpath ..\Aulas_Praticas\lib\
addpath 'Robô FANUC - Material de apoio-20221212'\FANUC-RobComm\fanuctoolbox\
load cameraParams_V3.mat
load 'robo_zero_pattern.mat'
trans_patern_to_robot=trans(Pos(1,1),Pos(1,2),0)*rotz(pi/2)*rotx(pi); %transformação geometrica entre o zero da camara(canto xadrez) e o valor no referencial do robô desse mesmo ponto

altura_peca=180; %Diferenca entre a altura 0 e a altura das pecas vermelhas, azuis e verdes
altura_brown=110; %Diferenca entre a altura 0 e a altura das pecas castanhas 

robCOMM = Comm_Open(4900,'192.168.0.229');      %IP of the FANUC [LR MATE 200ID - SMALL ONE]
pause(2); %para esperar que a comunicacao se estabeleca
Mov_Joints(robCOMM,0,-51,28,1,-103,344,0,1000,1); %Move para o ponto de seguranca

%Verifica se ja chegou à posição
act_pos=Get_Cart_Abs(robCOMM);
while abs(act_pos(1,3)-273.1960)>0.1
    act_pos=Get_Cart_Abs(robCOMM);
    pause(1); 
end 

cam = webcam(1); %inicia a camara
cam.Resolution="1280x960"; %define a resolucao a usar
pause(5); %para a imagem estabilizar
img = snapshot(cam); %tira foto

% img=imread("snapshot4.png"); 

img = undistortImage(img, cameraParams_v3); %corrige a imagem
figure(1)
subplot(121)
imshow(img);

[corner_pix, border_size] = detectCheckerboardPoints(img); %deteta o tabuleiro de xadrez
hold on
plot(corner_pix(1,1),corner_pix(1,2), 'og') %Faz plot do (0,0) da camara

% gerar pontos no referêncial
points_pattern = generateCheckerboardPoints(border_size,24);

% matriz de rotação e vetor de translação
% diagonal de R a 1 indicava alinhamento paralelo
[R,t] = extrinsics(corner_pix, points_pattern, cameraParams_v3);

% pixels para o mundo
% projetar os pontos dos pixels no mundo real
points_world = pointsToWorld(cameraParams_v3,R,t,corner_pix);

%Mudar a imagem para HSV para ser mais facil distinguir as cores
img_hsv = rgb2hsv(img);
H = img_hsv(:,:,1);
S = img_hsv(:,:,2);
V = img_hsv(:,:,3);

%RED
maskred = (H>=0.929 | H<=0.076) &  (S>0.33); %& (V>0.555)  %Cria uma mascara que isola a cor vermelha
subplot(122)
maskred = bwareaopen(maskred,800); %elimina os objetos com menos de 800 pixeis
maskred = imfill(maskred,"holes"); %Enche os objetos para melorar o calculo do region props
props_red = regionprops(maskred, 'Centroid','Area','Orientation','MaxFeretProperties','MinFeretProperties','BoundingBox'); %calcula as propriedades
mask=imoverlay(maskred,maskred,'red'); %Faz display da mascara
imshow(mask);

hold on

%BLUE
maskblue = (H>0.47)& (H<0.765) & (S>=0.239) & (V>0.446); %Cria uma mascara que isola a cor azul
subplot(122)
maskblue = bwareaopen(maskblue,800); %elimina os objetos com menos de 800 pixeis
maskblue = imfill(maskblue,"holes"); %Enche os objetos para melorar o calculo do region props
props_blue = regionprops(maskblue, 'Centroid','Area','Orientation','MaxFeretProperties','MinFeretProperties','BoundingBox'); %calcula as propriedades
mask=imoverlay(mask,maskblue,'blue'); %Faz display da mascara
imshow(mask);

%GREEN
maskgreen = (H>0.212)& (H<0.483) & (S>0.274)& (V>0.333); %Cria uma mascara que isola a cor verde
subplot(122)
maskgreen = bwareaopen(maskgreen,800); %elimina os objetos com menos de 800 pixeis
maskgreen = imfill(maskgreen,"holes"); %Enche os objetos para melorar o calculo do region props
props_green = regionprops(maskgreen, 'Centroid','Area','Orientation','MaxFeretProperties','MinFeretProperties','BoundingBox'); %calcula as propriedades
mask=imoverlay(mask,maskgreen,'green'); %Faz display da mascara
imshow(mask);

%BROWN
maskbrown = (H>0.078)& (H<0.131) & (S>0.159);
subplot(122)
maskbrown = bwareaopen(maskbrown,800); %eli minaos objetos com menos de 800 pixeis
maskbrown = imfill(maskbrown,"holes"); %Enche os objetos para melorar o calculo do region props
props_brown = regionprops(maskbrown, 'Centroid','Area','Orientation','MaxFeretProperties','MinFeretProperties','BoundingBox'); %calcula as propriedades
mask=imoverlay(mask,maskbrown,'w'); %Faz display da mascara
imshow(mask);


if ~isempty(props_red) %Se houver pecas vermelhas
    for i=1:size(props_red) 
        subplot(121);
        plot(props_red(i).Centroid(:,1),props_red(i).Centroid(:,2), 'Marker','+','MarkerSize',10,'Color','yellow') %Faz display dos centroids das pecas
        %Coloca em variaveis os parametros do region props
        p_centriod_red(i,:) = props_red(i).Centroid; 
        p_area_red(i) = props_red(i).Area;
        p_orientation_red(i) = props_red(i).Orientation;
        p_maxFeret_red(i)= props_red(i).MaxFeretDiameter; 
        p_minFeret_red(i)= props_red(i).MinFeretDiameter; 
        p_bounding_red(i,:)= props_red(i).BoundingBox; 
    end

    %descobre o index do item com maior area e guarda na variavel y
    maximum = max(max(p_area_red));
    [x,y]=find(p_area_red==maximum); 
    
    dist=p_bounding_red(y,4)/(length(props_red)-1); %calucula a distancia de colocacao das peças na zona de maior area

    count=0; %Contador auxiliar para colocar as pecas no ponto certo
    for j=1:size(props_red) %Vai buscar todas as peças que existirem vermelhas e colocalas na folha grande
        if j==y %se for a zona de maior area continua 
            continue;
        end     
        
        %Calcula os pontos de colocacao na folha
        ponto_colocacaox=p_centriod_red(y,1);
        ponto_colocacaoy=(p_centriod_red(y,2)-(p_bounding_red(y,4)/2)+dist/2+dist*count);
        count=count+1;

        ponto=[ponto_colocacaox ponto_colocacaoy]'; %Concatenar as coordenadas do ponto

        %Rotacao dos pontos conforme a orientacao da folha 
        rotac=Rot(deg2rad(-p_orientation_red(y)+90)); 
        ponto_colocacao = rotac*(ponto - p_centriod_red(y,:)') + p_centriod_red(y,:)'; 

        plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','white') %Faz plot do ponto de colocacao de cada peca 
        
        %calcula o valor que o punho do robot tem que rodar para ficar paralelo com a folha
        if p_orientation_red(j)>0 
            if p_orientation_red(y)>0
                orientacao=((p_orientation_red(y))-(p_orientation_red(j)))-90; 
            else 
                orientacao=((p_orientation_red(y))-(p_orientation_red(j)))+90;
            end
        else 
            if p_orientation_red(y)>0
                orientacao=((p_orientation_red(y))-(p_orientation_red(j)))-90;
            else
                orientacao=((p_orientation_red(y))-(p_orientation_red(j)))+90; 
            end
        end
        
        %Bucar a peça ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_red(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Se tiver 1 quer dizer que consegue fazer o movimento. Se for 9 o robô nao consegue chegar  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Buscar peça mesa
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_red(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Close(robCOMM,2);
        pause(1);

        %Ponto de desaproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_red(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        
        %Levar a peça à folha ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Levar a peça à folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,orientacao,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Open(robCOMM,2);
        pause(1);

        %Ponto de desaproximação folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
    end 
end







if ~isempty(props_blue)
    for i=1:size(props_blue)
        subplot(121);
        plot(props_blue(i).Centroid(:,1),props_blue(i).Centroid(:,2), 'Marker','+','MarkerSize',10,'Color','yellow')
        p_centriod_blue(i,:) = props_blue(i).Centroid;
        p_area_blue(i) = props_blue(i).Area;
        p_orientation_blue(i) = props_blue(i).Orientation;
        p_maxFeret_blue(i)= props_blue(i).MaxFeretDiameter; 
        p_minFeret_blue(i)= props_blue(i).MinFeretDiameter; 
        p_bounding_blue(i,:)= props_blue(i).BoundingBox; 
    end

    maximum = max(max(p_area_blue));
    [x,y]=find(p_area_blue==maximum); %descobre o index do item com maior area
    
    dist=p_bounding_blue(y,4)/(length(props_blue)-1); %distancia entre as peças na zona de maior area

    count=0; 
    for j=1:size(props_blue) %Vai buscar todas as peças que existirem vermelhas e colocalas na folha branca
        if j==y 
            continue;
        end     
         
        ponto_colocacaox=p_centriod_blue(y,1);%*abs(sind(p_orientation_blue(y)));
        ponto_colocacaoy=(p_centriod_blue(y,2)-(p_bounding_blue(y,4)/2)+dist/2+dist*count);%*abs(sind(p_orientation_blue(y)));
        count=count+1;

        ponto=[ponto_colocacaox ponto_colocacaoy]'; 

        rotac=Rot(deg2rad(-p_orientation_blue(y)+90)); 
         
        ponto_colocacao = rotac*(ponto - p_centriod_blue(y,:)') + p_centriod_blue(y,:)'; 
        if count==1
            plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','red')
        else
            plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','white')
        end
        
        if p_orientation_blue(j)>0 %calcula a orientação 
            if p_orientation_blue(y)>0
                orientacao=((p_orientation_blue(y))-(p_orientation_blue(j)))-90; 
            else 
                orientacao=((p_orientation_blue(y))-(p_orientation_blue(j)))+90;
            end
        else 
            if p_orientation_blue(y)>0
                orientacao=((p_orientation_blue(y))-(p_orientation_blue(j)))-90;
            else
                orientacao=((p_orientation_blue(y))-(p_orientation_blue(j)))+90; 
            end
        end
        
        %Bucar a peça ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_blue(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Buscar peça mesa
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_blue(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Close(robCOMM,2);
        pause(1);

        %Ponto de desaproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_blue(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        
        %Levar a peça à folha ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Levar a peça à folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,orientacao,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Open(robCOMM,2);
        pause(1);

        %Ponto de desaproximação folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
    end 
end








if ~isempty(props_green)
    for i=1:size(props_green)
        subplot(121);
        plot(props_green(i).Centroid(:,1),props_green(i).Centroid(:,2), 'Marker','+','MarkerSize',10,'Color','yellow')
        p_centriod_green(i,:) = props_green(i).Centroid;
        p_area_green(i) = props_green(i).Area;
        p_orientation_green(i) = props_green(i).Orientation;
        p_maxFeret_green(i)= props_green(i).MaxFeretDiameter; 
        p_minFeret_green(i)= props_green(i).MinFeretDiameter; 
        p_bounding_green(i,:)= props_green(i).BoundingBox; 
    end

    maximum = max(max(p_area_green));
    [x,y]=find(p_area_green==maximum); %descobre o index do item com maior area
    
    dist=p_bounding_green(y,4)/(length(props_green)-1); %distancia entre as peças na zona de maior area

    count=0; 
    for j=1:size(props_green) %Vai buscar todas as peças que existirem vermelhas e colocalas na folha branca
        if j==y 
            continue;
        end     
         
        ponto_colocacaox=p_centriod_green(y,1);%*abs(sind(p_orientation_green(y)));
        ponto_colocacaoy=(p_centriod_green(y,2)-(p_bounding_green(y,4)/2)+dist/2+dist*count);%*abs(sind(p_orientation_green(y)));
        count=count+1;

        ponto=[ponto_colocacaox ponto_colocacaoy]'; 

        rotac=Rot(deg2rad(-p_orientation_green(y)+90)); 
         
        ponto_colocacao = rotac*(ponto - p_centriod_green(y,:)') + p_centriod_green(y,:)'; 
        if count==1
            plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','red')
        else
            plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','white')
        end
        
        if p_orientation_green(j)>0 %calcula a orientação 
            if p_orientation_green(y)>0
                orientacao=((p_orientation_green(y))-(p_orientation_green(j)))-90; 
            else 
                orientacao=((p_orientation_green(y))-(p_orientation_green(j)))+90;
            end
        else 
            if p_orientation_green(y)>0
                orientacao=((p_orientation_green(y))-(p_orientation_green(j)))-90;
            else
                orientacao=((p_orientation_green(y))-(p_orientation_green(j)))+90; 
            end
        end
        
        %Bucar a peça ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_green(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Buscar peça mesa
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_green(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Close(robCOMM,2);
        pause(1);

        %Ponto de desaproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_green(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        
        %Levar a peça à folha ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Levar a peça à folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,orientacao,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Open(robCOMM,2);
        pause(1);

        %Ponto de desaproximação folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
    end 
end






if ~isempty(props_brown)
    for i=1:size(props_brown)
        subplot(121);
        plot(props_brown(i).Centroid(:,1),props_brown(i).Centroid(:,2), 'Marker','+','MarkerSize',10,'Color','yellow')
        p_centriod_brown(i,:) = props_brown(i).Centroid;
        p_area_brown(i) = props_brown(i).Area;
        p_orientation_brown(i) = props_brown(i).Orientation;
        p_maxFeret_brown(i)= props_brown(i).MaxFeretDiameter; 
        p_minFeret_brown(i)= props_brown(i).MinFeretDiameter; 
        p_bounding_brown(i,:)= props_brown(i).BoundingBox; 
    end

    maximum = max(max(p_area_brown));
    [x,y]=find(p_area_brown==maximum); %descobre o index do item com maior area
    
    dist=p_bounding_brown(y,4)/(length(props_brown)-1); %distancia entre as peças na zona de maior area

    count=0; 
    for j=1:size(props_brown) %Vai buscar todas as peças que existirem vermelhas e colocalas na folha branca
        if j==y 
            continue;
        end     
         
        ponto_colocacaox=p_centriod_brown(y,1);%*abs(sind(p_orientation_brown(y)));
        ponto_colocacaoy=(p_centriod_brown(y,2)-(p_bounding_brown(y,4)/2)+dist/2+dist*count);%*abs(sind(p_orientation_brown(y)));
        count=count+1;

        ponto=[ponto_colocacaox ponto_colocacaoy]'; 

        rotac=Rot(deg2rad(-p_orientation_brown(y)+90)); 
         
        ponto_colocacao = rotac*(ponto - p_centriod_brown(y,:)') + p_centriod_brown(y,:)'; 
        if count==1
            plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','red')
        else
            plot(ponto_colocacao(1),ponto_colocacao(2), 'Marker','+','MarkerSize',5,'Color','white')
        end
        
        if p_orientation_brown(j)>0 %calcula a orientação 
            if p_orientation_brown(y)>0
                orientacao=((p_orientation_brown(y))-(p_orientation_brown(j)))-90; 
            else 
                orientacao=((p_orientation_brown(y))-(p_orientation_brown(j)))+90;
            end
        else 
            if p_orientation_brown(y)>0
                orientacao=((p_orientation_brown(y))-(p_orientation_brown(j)))-90;
            else
                orientacao=((p_orientation_brown(y))-(p_orientation_brown(j)))+90; 
            end
        end
        
        %Bucar a peça ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_brown(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Buscar peça mesa
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_brown(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Close(robCOMM,2);
        pause(1);

        %Ponto de desaproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,p_centriod_brown(j,:));
        grasping_point=trans_patern_to_robot*[centoid_red_world';0;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        
        %Levar a peça à folha ponto de aproximação
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_brown-200;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end

        %Levar a peça à folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_peca-altura_brown;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,orientacao,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
        Gripper_Open(robCOMM,2);
        pause(1);

        %Ponto de desaproximação folha
        centoid_red_world = pointsToWorld(cameraParams_v3,R,t,ponto_colocacao');
        grasping_point=trans_patern_to_robot*[centoid_red_world';altura_brown-200;1]; 
        S=Mov_Cart_Abs(robCOMM,grasping_point(1,1),grasping_point(2,1),grasping_point(3,1),180,0,0,0,1,1,0,0,0,1,1000,1); 
        if contains(S,'1') %Verificar o output do Status  
            act_pos=Get_Cart_Abs(robCOMM);
            while abs(act_pos(1,3)-grasping_point(3,1))>0.1 || abs(act_pos(1,2)-grasping_point(2,1))>0.1 || abs(act_pos(1,1)-grasping_point(1,1))>0.1
                act_pos=Get_Cart_Abs(robCOMM);
                pause(0.5); 
            end
         else
             disp("O ponto está fora do alcance do robô")
             continue; 
        end
    end 
end





Mov_Joints(robCOMM,0,-51,28,1,-103,344,0,1000,1); %Vai para posicao seguranca

    


clear cam;

