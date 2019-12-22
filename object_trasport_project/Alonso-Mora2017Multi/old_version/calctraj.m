function [zarray, dzarray, tarray] = calctraj(via,dt,tacc)
ptnum = size(via,2);
z0 = via(1,:);
zarray = mstraj(via(2:end,:),ones(1,ptnum),[],z0,dt,tacc); % row vector for each xr
zarray = [z0;zarray];
dzarray = [diff(zarray)./dt;zeros(1,ptnum)];
tarray = 0:dt:(size(zarray,1)-1)*dt;
end