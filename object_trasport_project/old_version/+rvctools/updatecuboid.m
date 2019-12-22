function cub = updatecuboid(cub,xc)
import rvctools.*
[R,t] = tr2rt(x2T(xc));
cub.vertw = (R*cub.vertb' + t)';
end