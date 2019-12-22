function [y,dy] = getviatnow(yarray, dyarray, tarray, tnow)
y = interp1(tarray,yarray,tnow); 
dy = interp1(tarray,dyarray,tnow);
end