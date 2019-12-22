function [xr,xc,xm] = z2x(z,lc,lm)
if length(z)==12
    xr = [z(1:2),0,0,0,z(3)]; a = z(4:7); th = z(8:11); w = z(12);
else   
    xr = [z(1:2),0,0,0,0]; a = z(3)*ones(1,4); th = zeros(1,4); w = z(4);
end
xc = xr2c(xr,w);
xm = xr2m(xr,a,th,lc,lm);
end