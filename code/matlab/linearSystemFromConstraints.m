function [A,b] = linearSystemFromConstraints(c,x)
    xcount = length(x);
    ccount = length(c);
    A = repmat(x(1),[ccount,xcount]);
    A(:) = 0;
    b = repmat(x(1),[ccount,1]);
    b(:) = 0;
    for j=1:ccount
        b(j) = c(j);
        for i=1:xcount
            cc = coeffs(c(j),x(i));
            if(length(cc)>1)
                A(j,i) = cc(2);
                b(j) = b(j) - A(j,i)*x(i);
            end
        end
    end
end