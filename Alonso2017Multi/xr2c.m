function xc0 = xr2c(xr0,w0)
Tr2c0 = transl([0,0,w0]);
xc0 = T2x(x2T(xr0)*Tr2c0);
end