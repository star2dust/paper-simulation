function dy = ydot(t,y,zarray,dzarray,tarray,fd,mi,mc,lc)
xxi = reshape(y(1:8),2,4);
dxxi = reshape(y(9:16),2,4);
xxc = y(17:18);
dxxc = y(19:20);
xc = [xxc',0,0,0,0];
% get current z (z in planning)
z = interp1(tarray,zarray,t); 
dz = interp1(tarray,dzarray,t);
% contact points (zi in paper)
Tc2e = lc2Tce(lc);
for i=1:4
    xe = T2x(x2T(xc)*Tc2e{i})';
    ai(:,i) = xe(1:2)';
    zi(:,i) = xxi(:,i)-ai(:,i); % eq (3)
    k = 1; k1 = 1; k2 = 1;
    fi(:,i) = k*zi(:,i)+k1*zi(:,i)'*zi(:,i)*zi(:,i); % eq (37) in paper 2009
    vd(:,i) = (dz(1:2) - k2*(xc(1:2)-z(1:2)))'; % desired velocity
    G = eye(2); % Gamma
    Fi(:,i) = -G*(dxxi(:,i)-vd(:,i))+fd(:,i); % eq (14)
    ddxxi(:,i) = (Fi(:,i)-fi(:,i))./mi(i); % eq (7)
end
ddxxc = sum(fi,2)./mc; % eq(8)
dy = [dxxi(:);ddxxi(:);dxxc;ddxxc];
end