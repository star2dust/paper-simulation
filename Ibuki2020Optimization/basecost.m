function [g,ng,png_pt,hg] = basecost(pb,th,fk,jac,t,pe,T,thr,dpe,dT,dthr)
% Cost function for joint and base control
if nargin<11
   dthr = zeros(size(thr));
   if nargin<10
       dT = zeros(size(T));
       if nargin<9
           dpe = zeros(size(pe));
       end
   end
end
% state for one robot
th = th(:);
pb = pb(:);
fk = fk(:);
thr = thr(:);
dthr = dthr(:);
pe = pe(:);
dpe = dpe(:);
% weight for joint angle
W = blkdiag(0.5,eye(length(th)-1))*5;
dW = zeros(size(W));
g = (th-thr)'*W*(th-thr)/2+(pb+fk-pe)'*(T'*T)*(pb+fk-pe)/2;
ng = [T'*T*(pb+fk-pe);jac'*(T'*T)*(pb+fk-pe)+W'*(th-thr)];
png_pt = [dT'*T*(pb+fk-pe)+T'*dT*(pb+fk-pe)-T'*T*dpe;
    jac'*(dT'*T)*(pb+fk-pe)+jac'*(T'*dT)*(pb+fk-pe)-jac'*(T'*T)*dpe+dW'*(th-thr)-W'*dthr];
hg = [T'*T,T'*T*jac;jac'*(T'*T),jac'*(T'*T)*jac+W'];
end