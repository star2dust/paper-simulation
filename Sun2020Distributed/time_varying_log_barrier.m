close all
clear

f = -5:0.01:0;
t = 0:0.01:5;
rho = exp(t);
plot(t,rho);
figure
z = -log(rho)./rho;
plot(t,z);
figure
x = log(t);
plot(t,x);
figure
phi = -1./rho'*log(1./rho-f);
plot(f',phi');
figure
phi = -1./rho'*log(1-rho.*f);
% phi = -1./rho'*(log(1./rho-f)+log(rho));
plot(ones(1,size(phi,1)).*f',phi');
figure
[x,y] = meshgrid(0:0.1:10,-10:0.1:0);
pof = -1./exp(x).*log(-exp(x).*y);
mesh(x,y,pof)