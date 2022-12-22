function MDH = GenerateMultiDH(DH,MQ,t)

% MDH - hipermatriz de matrizes DH definidas para os diversos vetores coluna de MQ
% DH - A matriz base de Denavit-Hartenberg que corresponde à posicao
% zero do robo (juntas no valor de home position)
% MQ - dado por Linespacevect(Qi, Qf, N)
% Qi e Qf - vetores dos valores iniciais e finais das juntas
% N - número de colunas de MQ, i.e., número de posicoes a calcular.

MDH = zeros(size(DH,1), 4, size(MQ,2));

if nargin == 3
    for n=1:size(MQ,2)
        for i=1:size(MQ,1)
            if t(i)==0
                MDH(i,:,n) = [DH(i,1)+MQ(i,n) DH(i,2:end)];
            else
                MDH(i,:,n) = [DH(i,1) DH(i,2) DH(i,3)+MQ(i,n) DH(i,4)];
            end
        end
    end
else
     for n=1:length(MQ)
        for i=1:size(MQ,1)
           MDH(i,:,n) = [DH(i,1)+MQ(i,n) DH(i,2:end)];
        end
     end
end