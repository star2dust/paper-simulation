function [yarray0, dyarray0, tarray] = calctraj(via,dt,tacc)
ptlen = size(via,2);
y0 = via(1,:);
yarray0 = mstraj(via(2:end,:),ones(1,ptlen),[],y0,dt,tacc); % row vector for each xr
yarray0 = [y0;yarray0];
dyarray0 = [diff(yarray0)./dt;zeros(1,ptlen)];
tarray = 0:dt:(size(yarray0,1)-1)*dt;
end