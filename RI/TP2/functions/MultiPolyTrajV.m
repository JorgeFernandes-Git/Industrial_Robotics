function [QQ,t] = MultiPolyTrajV(Q,N,tt,vmed)

% Retorna trajetoria polinomial multipla entre duas posturas com velocidades
% iniciais e nais nulas e passando por pontos intermedios (via points).
% QQ - matriz que, em cada coluna, tem os valores das juntas nos instantes
% correspondentes, incluindo os via points.
% t - vetor com os instantes de tempo correspondentes a todos os instantes de
% amostragem da trajetoria do incio ao m.
% Par^ametros da func~ao
% Q - matriz com os vetores iniciais, nais e intermedios das posic~oes a
% percorrer: [ Qi QA QB ... Qf]
% tt - vetor com os instantes nais de cada via point (espera-se que o
% instante inicial e 0, mas podera n~ao ser...)
% N - vetor com os numeros de pontos a usar em cada sub-trajetoria.

V = zeros(size(Q));
QQ = [];
t = [];

if vmed == 1
    for i=2:size(Q,2)-1
        dq_1 = (Q(:,i)-Q(:,i-1))/(tt(i)-tt(i-1));
        dq_2 = (Q(:,i+1)-Q(:,i))/(tt(i+1)-tt(i));
        m = sign(dq_1) == sign(dq_2);
        V(m,i) = (dq_1(m)+dq_2(m))/2;
    end
end

for i=1:size(Q,2)-1
    Qi = Q(:,i);
    Qf = Q(:,i+1);

    t0 = tt(i);
    tf = tt(i+1);

    Vi = V(:,i);
    Vf = V(:,i+1);

    [q,ttt]=PolyTrajV(Qi,Qf,Vi,Vf,N,t0,tf);
    QQ = [QQ q];
    t = [t ttt];
end