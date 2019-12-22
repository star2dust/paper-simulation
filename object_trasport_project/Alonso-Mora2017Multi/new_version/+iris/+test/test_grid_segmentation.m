function test_grid_segmentation()
load('data/example_feas_map')
grid = ~Q(85:125,25:85);
% grid = Q;

figure(16)
clf;
imshow(grid, 'InitialMagnification', 'fit');
hold on;

% profile on
t0 = tic;
obstacles = iris.terrain_grid.segment_grid(grid);
toc(t0);
% profile viewer

figure(16)
for j = 1:length(obstacles)
  obs = obstacles{j};
  plot(obs(2,:), obs(1,:), 'r-');
end

end

