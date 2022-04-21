function [g,ng,png_pt,hg] = logbarrier(x,t,A,b,dA,db)

if nargin<5
   dA = zeros(size(A));
   db = zeros(size(b));
end
x = x(:);
% set rho parameters (想离边界远可设a2=1e-3，想允许一定的误差可设k1=1)
a1 = 1; a2 = 1e-3; k1 = 0.1; 
rho = @(t) a1*exp(a2*t); drho = @(t) a2*rho(t);
g = -ones(1,size(A,1))*log(k1-rho(t)*(A*x-b))/rho(t);
ng = A'*(1./(k1-rho(t)*(A*x-b)));
png_pt = dA'*(1./(k1-rho(t)*(A*x-b)))+...
    A'*(drho(t)*(A*x-b)./(k1-rho(t)*(A*x-b)).^2)+...
    A'*(rho(t)*(dA*x-db)./(k1-rho(t)*(A*x-b)).^2);
hg = zeros(length(x));
for i=1:size(A,1)
    hg = hg+A(i,:)'*A(i,:)*(rho(t)./(k1-rho(t)*(A(i,:)*x-b(i,:))).^2);
end
% standard barrier function
% g = -ones(1,size(A,1))*log(1-rho(t)*(A*x-b))/rho(t);
% ng = A'*(1./(1-rho(t)*(A*x-b)));
% png_pt = dA'*(1./(1-rho(t)*(A*x-b)))+...
%     A'*(drho(t)*(A*x-b)./(1-rho(t)*(A*x-b)).^2)+...
%     A'*(rho(t)*(dA*x-db)./(1-rho(t)*(A*x-b)).^2);
% hg = A'*A*(ones(1,size(A,1))*(rho(t)./(1-rho(t)*(A*x-b)).^2));
end