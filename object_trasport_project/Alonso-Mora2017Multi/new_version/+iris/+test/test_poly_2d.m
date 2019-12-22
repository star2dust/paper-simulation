function results = test_poly_2d(record)
import iris.inflate_region;
import iris.drawing.*;

if nargin < 1
  record = false;
end

lb = [0;0];
ub = [10;10];
dim = 2;

n_obs = 20;
obstacle_pts = iris.test.random_obstacles(dim, n_obs, lb, ub, 5);

A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-lb; ub];

start = 0.5 * (ub + lb);

% profile on
[A,b,C,d,results] = inflate_region(obstacle_pts, A_bounds, b_bounds, start);
% profile viewer
if n_obs < 1e4
  animate_results(results, record);
end

end
