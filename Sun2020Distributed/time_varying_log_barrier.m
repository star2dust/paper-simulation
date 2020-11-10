close all
clear

f = -5:0.01:0;
t = 0:0.1:10;
rho = exp(t);
plot(t,rho);
figure
phi = -1./rho'*log(-f);
plot(f,phi);
figure
[x,y] = meshgrid(0:0.1:10,-10:0.1:0);
pof = -1./exp(x).*log(-exp(x).*y);
mesh(x,y,pof)