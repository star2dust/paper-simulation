A_bounds = [-eye(3); 
            1,1,1];
b_bounds = [0;0;0;1];
lb = [0;0;0];
ub = [1;1;1];

[C, d] = iris.inner_ellipsoid.mosek_nofusion(A_bounds, b_bounds);
assert(det(C) > 0.01);

iris.drawing.draw_3d(A_bounds, b_bounds, C, d, [], lb, ub);
