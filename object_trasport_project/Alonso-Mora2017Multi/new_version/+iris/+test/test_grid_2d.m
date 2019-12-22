import iris.inflate_region;
import iris.terrain_grid.*;
import iris.thirdParty.polytopes.*;

load('data/example_feas_map.mat');
grid = Q(85:125,25:85);
dists = obs_dist(grid);
[~, i0] = max(reshape(dists, [], 1));

[r, c] = ind2sub(size(grid), i0);
start = [r;c];


[white_squares(1,:), white_squares(2,:)] = ind2sub(size(grid), find(grid));
[black_squares(1,:), black_squares(2,:)] = ind2sub(size(grid), find(~grid));
black_edges = [];
[black_edges(1,:), black_edges(2,:)] = ind2sub(size(grid), find(component_boundary(grid, [r;c])));

% offsets = [-0.5,-0.5,0.5,0.5;-0.5,0.5,0.5,-0.5];
offsets = [0;0];
obstacles = {};
for j = 1:size(black_edges,2)
  obstacles{j} = bsxfun(@plus, black_edges(:,j), offsets);
end
obstacle_pts = reshape(cell2mat(obstacles), 2, 1, []);

lb = [0;0];
ub = [size(grid,1); size(grid,2)];
A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-lb; ub];

% profile on
[A,b,C,d] = inflate_region(obstacle_pts, A_bounds, b_bounds, start);
% profile viewer

figure(2)
clf
hold on
for j = 1:length(obstacles)
  patch(obstacles{j}(1,:), obstacles{j}(2,:), 'k');
  end
V = lcon2vert(A, b);
k = convhull(V(:,1), V(:,2));
plot(V(k,1), V(k,2), 'ro-');
th = linspace(0,2*pi,100);
y = [cos(th);sin(th)];
x = bsxfun(@plus, C*y, d);
plot(x(1,:), x(2,:), 'b-');
xlim([lb(1),ub(1)])
ylim([lb(1),ub(2)])