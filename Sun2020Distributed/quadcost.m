function [g,ng,png_pt,hg] = quadcost(x,t,A,b,dA,db)
% Cost function for quadratic program
if nargin<5
   dA = zeros(size(A));
   db = zeros(size(b));
end
x = x(:); 
b = b(:);
db = db(:);
% postive definite weight matrix
W = eye(size(A,1)); dW = zeros(size(W));
g = (A*x-b)'*W*(A*x-b)/2;
ng = A'*W'*(A*x-b);
png_pt = dA'*W'*(A*x-b)+A'*dW'*(A*x-b)+A'*W*(dA*x-db);
hg = A'*W'*A;

end