function [mask, A, b, obs] = inflate_grid_region(~, seed_grid)
import iris.terrain_grid.*;
import iris.inflate_region;

dim = 2;

grid = logical(seed_grid);
[all_squares(1,:), all_squares(2,:)] = ind2sub(size(grid), 1:length(reshape(grid,[],1)));

dists = obs_dist(seed_grid);
[~, i0] = max(reshape(dists, [], 1));
[r, c] = ind2sub(size(grid), i0);
x0 = [r; c];

[white_squares(1,:), white_squares(2,:)] = ind2sub(size(grid), find(grid));
[black_squares(1,:), black_squares(2,:)] = ind2sub(size(grid), find(~grid));
black_edges = [];
[black_edges(1,:), black_edges(2,:)] = ind2sub(size(grid), find(component_boundary(grid, [r;c])));

obs_offsets = [0.5, 0.5, -0.5, -0.5;
                   -0.5, 0.5, 0.5, -0.5];
obs_centers = reshape(black_edges, [], 1);
n_obs = size(black_edges, 2);
obs_pts = bsxfun(@plus, obs_centers, repmat(obs_offsets, n_obs, 1));
obstacles = mat2cell(obs_pts, dim*ones(n_obs,1), size(obs_offsets,2))';
obstacle_pts = reshape(cell2mat(obstacles), 2, 4, []);
% obstacles = mat2cell(black_edges, 2, ones(1,size(black_edges,2)));

lb = [0;0];
ub = [size(grid,1); size(grid,2)];
A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-lb; ub];

[A,b,C,d] = inflate_region(obstacle_pts, A_bounds, b_bounds, x0);


% for i = 1:size(A,2)
%   n = norm(A(i,:));
%   A(i,:) = A(i,:) / n;
%   b(i) = b(i) / n;
% end
mask = false(size(grid));
inpoly = all_squares(:,all(bsxfun(@minus, A * all_squares, b) <= 0));
mask(sub2ind(size(grid), inpoly(1,:), inpoly(2,:))) = true;
try
  k = convhull(inpoly(1,:), inpoly(2,:), 'simplify', true);
catch
  k = 1:size(inpoly, 2);
end
obs = inpoly(:,k);

% 
% mask = zeros(size(grid));
% mask(all(bsxfun(@minus, A * all_squares, b) < 0.5, 1)) = 1;
% mask(~grid) = 0;
