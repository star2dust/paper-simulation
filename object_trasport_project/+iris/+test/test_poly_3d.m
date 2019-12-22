function results = poly_segmentation_3d(record)
if nargin < 1
  record = false;
end
import iris.drawing.*;

lb = [0;0;0];
ub = [10;10;10];
dim = 3;

n_obs = 20;
% obstacles = iris.test.random_obstacles(dim, n_obs, lb, ub,3);
obstacles = iris.test.random_cubic_obstacles(dim, n_obs, lb, ub);

A_bounds = [-1,0,0;
            0,-1,0;
            0,0,-1;
            1,0,0;
            0,1,0;
            0,0,1];
b_bounds = [-lb;ub];

start = 0.5 * (ub + lb);


% profile on
[A,b,C,d,results] = iris.inflate_region(obstacles, A_bounds, b_bounds, start);
% profile viewer
if n_obs < 50
  animate_results(results, record);
end

end
