function MQ = LinspaceVect(Qi,Qf,N)

% Qi - vetor dos valores iniciais
% Qf - vetor dos valores finais
% N - número de elementos dos linspace
% MQ - matriz com todos os vetores - cada linha será o linspace dos
% elementos correspondentes de Qi até Qf.

for n=1:length(Qi)
    MQ(n,:) = linspace(Qi(n), Qf(n), N);
end
