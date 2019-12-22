function results = test_c_space_3d(record)
import iris.cspace.cspace3;
import iris.cspace.project_c_space_region;
import iris.inflate_region;
import iris.drawing.*;

if nargin < 1
  record = false;
end

dim = 3;
% obstacles = {};
n_obs = 10;
lb = [0;0;-pi];
ub = [4;4;pi];
bot = [-0.005,-0.005,0.005,0.005;-0.3,0.3,0.3,-0.3];
A_bounds = [-1,0,0;
            0,-1,0;
            0,0,-1;
            1,0,0;
            0,1,0;
            0,0,1];
base_obstacles = iris.test.random_obstacles(2, n_obs, lb(1:2), ub(1:2));
base_obstacles = mat2cell(reshape(base_obstacles(:,1,:), 2, []), 2, ones(1, n_obs));
obstacle_pts = cspace3(base_obstacles, bot, 10);
b_bounds = [-lb;ub];
start = 0.5 * (lb + ub);

% profile on
[A,b,C,d,results] = inflate_region(obstacle_pts, A_bounds, b_bounds, start);
% profile viewer
% animate_results(results, record);
figure(4)
clf
hold on
for j = 1:length(base_obstacles)
  obs = base_obstacles{j};
  if size(obs, 2) > 2
    patch(obs(1,:), obs(2,:), 'k');
  else
    plot(obs(1,:), obs(2,:), 'ko-', 'MarkerFaceColor', 'k');
  end
end
x = iris.sample_convex_polyhedron(A,b,50);
for k = 1:size(x,2)
  R = iris.util.rotmat(x(3,k));
  bot_x = bsxfun(@plus, R * bot, x(1:2,k));
  patch(bot_x(1,:), bot_x(2,:), 'k');
end
axis equal
lb = lb - 0.3;
ub = ub + 0.3;
plot([lb(1),ub(1),ub(1),lb(1),lb(1)], [lb(2),lb(2),ub(2),ub(2),lb(2)], 'k--')
[inner_poly, outer_poly] = project_c_space_region(A,b);
patch(outer_poly(1,:), outer_poly(2,:), 'y', 'FaceAlpha', 0.5);
if ~isempty(inner_poly)
  patch(inner_poly(1,:), inner_poly(2,:), 'g', 'FaceAlpha', 0.5);
end
axis off
end