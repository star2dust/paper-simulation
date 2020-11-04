function [z,flag] = formationP2z(polytope,zd,range,lc,lm)
max_aw = 1;
min_aw = 0.45;
polynum = length(polytope);
% get the A and b from polytope
Ap = []; bp = [];
for i=1:polynum
    Ap = [Ap;polytope(i).A];
    bp = [bp;polytope(i).b];
end
if length(zd)==12
    % constraints al<=a<=au and thl<=th<=thu just use lb and ub
    lb = [range.lb;-pi;min_aw*ones(4,1);-pi/4*ones(4,1);min_aw]';
    ub = [range.ub;pi;max_aw*ones(4,1);pi/4*ones(4,1);max_aw]';
    % calculate z
    [z,~,flag] = fmincon(@(z) (z-zd)*(z-zd)',zd,[],[],[],[],lb,ub,@(z) mycon(z,Ap,bp,lc,lm));
else
    % constraints al<=a<=au and thl<=th<=thu just use lb and ub
    lb = [range.lb;min_aw*ones(2,1)]';
    ub = [range.ub;max_aw*ones(2,1)]';
    % calculate z
    [z,~,flag] = fmincon(@(z) (z-zd)*(z-zd)',zd,[],[],[],[],lb,ub,@(z) mycon2(z,Ap,bp,lc,lm));
end
end

function [c,ceq] = mycon(z,A,b,lc,lm)
import rvctools.*
max_aw = 1;
min_aw = 0.45;
xr = [z(1:2),0,0,0,z(3)]; a = z(4:7); th = z(8:11); w = z(12);
xm = xr2m(xr,a,th,lc,lm);
for i=1:size(xm,1)
    plat(i) = pkgMechanics.RigidCuboid(1,xm(i,:),lm);
    pm_v(:,:,i) = plat(i).vertices(1:4,:)';
    Rm(:,:,i) = rpy2r(xm(i,4:end));
    pw_m(:,i) = xm(i,1:3)';
    pw_v(:,4*i-3:4*i) = pw_m(:,i)+Rm(:,:,i)*pm_v(:,:,i);
end
for i=1:size(pw_v,2)
    c1(:,i) = A*pw_v(1:2,i)-b;
end
c2 = (a.^2+w.^2-max_aw^2)';
c3 = -(a.^2+w.^2-min_aw^2)';
c = [c1(:);c2(:);c3(:)];
ceq = 0;
end

function [c,ceq] = mycon2(z,A,b,lc,lm)
import rvctools.*
max_aw = 1;
min_aw = 0.45;
xr = [z(1:2),0,0,0,0]; a = z(3)*ones(1,4); th = zeros(1,4); w = z(4);
xm = xr2m(xr,a,th,lc,lm);
for i=1:size(xm,1)
    plat(i) = pkgMechanics.RigidCuboid(1,xm(i,:),lm);
    pm_v(:,:,i) = plat(i).vertices(1:4,:)';
    Rm(:,:,i) = rpy2r(xm(i,4:end));
    pw_m(:,i) = xm(i,1:3)';
    pw_v(:,4*i-3:4*i) = pw_m(:,i)+Rm(:,:,i)*pm_v(:,:,i);
end
for i=1:size(pw_v,2)
    c1(:,i) = A*pw_v(1:2,i)-b;
end
c2 = (a.^2+w.^2-max_aw^2)';
c3 = -(a.^2+w.^2-min_aw^2)';
c = [c1(:);c2(:);c3(:)];
ceq = 0;
end