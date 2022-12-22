function J = jacobianUR10inv(Qn,Lc,Le,Lf,Lg,Lh)

J = inv(jacobianUR10(Qn,Lc,Le,Lf,Lg,Lh));

for j=1:width(J)
    for i=1:height(J)
        if isnan(J(j,i))
            disp('Desired position results in a singularity!')
            return
        end
    end
end