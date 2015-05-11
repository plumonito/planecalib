syms SX SY real
for i=1:3
    for j=1:3
        name = sprintf('H%d%d',j,i);
        H(j,i) = sym(name,'real');
    end
end
H(3,3) = 1;
P = [0,0; SX,0; 0,SY; SX,SY]';
P(3,:) = 1;

HP = H*P;

for i=1:4
    for j=1:2
        name = sprintf('P%d%d',j,i);
        P2(j,i) = sym(name,'real');
    end
end
P2(3,:) = 1;

c=[];
for i=1:4
    cx = cross(HP(:,i),P2(:,i));
    if(isempty(c))
        c = cx(1:2);
    else
        c(end+1:end+2) = cx(1:2);
    end
end

c=expand(c);
x = reshape(H',1,9);
[A,b]=linearSystemFromConstraints(c,x(1:8));
sol=expand(A\b);
expand(sol)