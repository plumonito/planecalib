syms x y lambda


r2 = x*x+y*y;
factor = 1/(1+lambda*r2);
xd = x*factor;
yd = y*factor;

dxd_dx = (1+lambda*(y^2-x^2))*factor*factor;
dxd_dy = -(2*lambda*x*y)*factor*factor;
dxd_dlambda = -(x*r2)*factor*factor;

dyd_dx = -(2*lambda*x*y)*factor*factor;
dyd_dy = (1+lambda*(-y^2+x^2))*factor*factor;
dyd_dlambda = -(y*r2)*factor*factor;

simplify(diff(xd,x) - dxd_dx)
simplify(diff(xd,y) - dxd_dy)
simplify(diff(xd,lambda) - dxd_dlambda)

simplify(diff(yd,x) - dyd_dx)
simplify(diff(yd,y) - dyd_dy)
simplify(diff(yd,lambda) - dyd_dlambda)