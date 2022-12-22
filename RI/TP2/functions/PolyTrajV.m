function [QQ,t] = PolyTrajV(Q0,Qf,Qv0,Qvf,N,t0,tf)


% Retorna trajetorias polinomiais de terceira ordem entre duas posturas com
% velocidades iniciais e nais distintas.
% QQ - matriz com N colunas onde cada coluna contem os valores das juntas
% nos instantes correspondentes.
% t - vetor com os tempos correspondentes aos N instantes de amostragem da
% trajetoria entre o instante inicial (t0) e o nal (tf).
% Os par^ametros da func~ao s~ao:
% Q0 - vetor da posic~ao inicial das juntas (0)
% Qf - vetor da posic~ao nal das juntas (f )
% Qv0 - vetor da velocidade inicial das juntas (90)
% Qvf - vetor da velocidade nal das juntas (9f )
% t0 - instante inicial da trajetoria (segundos)
% tf - instante nal da trajetoria (segundos)
% N - numero de pontos a usar na denic~ao da trajetoria

a = Q0;
b = Qv0;
c = ((3/(tf-t0)^2)*(Qf-Q0)-((2/(tf-t0))*Qv0)-((1/(tf-t0))*Qvf));
d = (-(2/(tf-t0)^3)*(Qf-Q0)+(1/(tf-t0)^2)*(Qvf+Qv0));

t = linspace(t0,tf,N);

QQ = a + b.*(t-t0)+c.*(t-t0).^2+d.*(t-t0).^3;

