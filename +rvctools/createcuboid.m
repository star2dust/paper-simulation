function cub = createcuboid(xc,lc)
import rvctools.*
vertt = [0,0,0;0,1,0;1,1,0;1,0,0;0,0,1;0,1,1;1,1,1;1,0,1];
face = [1,2,3,4;5,6,7,8;1,2,6,5;3,4,8,7;1,4,8,5;2,3,7,6];
% ^ y axis
% | 6 % % 7 -> top
% | % 2 3 % -> bottom
% | % 1 4 % -> bottom
% | 5 % % 8 -> top
% -------> x axis
cub.vertb = [vertt(:,1)*lc(1)-lc(1)/2,vertt(:,2)*lc(2)-lc(2)/2,vertt(:,3)*lc(3)-lc(3)/2]; 
cub.face = face;
[R,t] = tr2rt(x2T(xc));
cub.vertw = (R*cub.vertb' + t)';
end