function test_simple_poly_2d(record)
import iris.inflate_region;
import iris.drawing.*;

if nargin < 1
  record = false;
end

lb = [0;0];
ub = [10;10];

obstacles = zeros(2, 4, 2);
obs_offsets = [3, 3, -3, -3;
                     -1.5, 1.5, 1.5, -1.5];
obstacles(:,:,1) = bsxfun(@plus, [3;1.5], obs_offsets);
obstacles(:,:,2) = bsxfun(@plus, [3;8.5], obs_offsets);


A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-lb; ub];

start = [3;5];

% profile on
[A,b,C,d,results] = inflate_region(obstacles, A_bounds, b_bounds, start);
% profile viewer
animate_results(results,record);

end
