function region = polytope(obstacle_pts,path_pts,range)
import iris.inflate_region
ptnum = size(path_pts,2);
A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-range.lb; range.ub];
region = struct;
for i=1:ptnum
    [A,b,C,d,results] = inflate_region(obstacle_pts, A_bounds, b_bounds, path_pts(:,i));
    region(i).A = A;
    region(i).b = b;
    region(i).C = C;
    region(i).d = d;
end
end
