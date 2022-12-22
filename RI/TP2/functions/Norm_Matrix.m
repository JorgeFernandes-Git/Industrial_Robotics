function [A,B] = Norm_Matrix(A,B)
[m,n] = size(A);
[m1,n1] = size(B);
if m < m1
    A(m1,:) = 0; % extend A with rows of zeros, up to the number of rows of A1
elseif m > m1
    B(m,:) = 0; % extend B with rows of zeros, up to the number of rows of A
end
if n < n1
    A(:,n1) = 0; % extend A with columns of zeros, up to the number of columns of A1
elseif n > n1
    B(:,n) = 0; % extend B with columns of zeros, up to the number of columns of A
end