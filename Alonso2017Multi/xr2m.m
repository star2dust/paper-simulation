function xm0 = xr2m(xr0,a0,th0,lc,lm)
lmz = lm(3);
n = length(th0);
[~,Tr2d] = lc2Tce(lc);
for i=1:n
    Td2m{i} = trotz(th0(i))*transl([a0(i),0,lmz/2])*trotz(pi);
    xm0(i,:) = T2x(x2T(xr0)*Tr2d{i}*Td2m{i});
end
end