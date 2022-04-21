function [q,qd,qdd,t] = mstraj_(qvia,qdmax,dt,tacc)
% Generate trajectory from via points
qlen = size(qvia,2);
q0 = qvia(1,:);
q = mstraj(qvia(2:end,:),qdmax,[],q0,dt,tacc); % row vector for each xr
q = [q0;q];
qd = [diff(q)./dt;zeros(1,qlen)];
qdd = [diff(qd)./dt;zeros(1,qlen)];
t = 0:dt:(size(q,1)-1)*dt;
end