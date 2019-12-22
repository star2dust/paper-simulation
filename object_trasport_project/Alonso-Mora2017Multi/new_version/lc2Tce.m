function [Tc2e, Tr2d] = lc2Tce(lc)
lcx = lc(1); lcy = lc(2);
Tr2d{1} = transl([-lcx/2,0,0])*trotz(pi);
Tr2d{2} = transl([0,-lcy/2,0])*trotz(-pi/2);
Tr2d{3} = transl([lcx/2,0,0]);
Tr2d{4} = transl([0,lcy/2,0])*trotz(pi/2);
Tc2e{1} = transl([-lcx/2,0,0])*rpy2tr([pi/2,0,pi/2]);
Tc2e{2} = transl([0,-lcy/2,0])*rpy2tr([pi/2,0,pi]);
Tc2e{3} = transl([lcx/2,0,0])*rpy2tr([pi/2,0,3*pi/2]);
Tc2e{4} = transl([0,lcy/2,0])*rpy2tr([pi/2,0,0]);
end