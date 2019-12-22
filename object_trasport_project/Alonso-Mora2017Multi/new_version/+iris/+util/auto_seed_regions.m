function safe_regions = auto_seed_regions(obstacle_pts, lb, ub, initial_seeds, num_regions, num_steps, callback)
% Automatically try to seed a set of IRIS regions in an environment. Begins
% by seeding at all of the initial_seeds values. Next, we grid the space by
% units of (ub - lb) / num_steps, locate the grid cell which is farthest from the set of
% obstacles and regions, and seed a new region there. We repeat until we have
% num_regions regions.

if isempty(callback)
  callback = @(r, s) false;
end
if iscell(obstacle_pts)
  padded = iris.pad_obstacle_points(obstacle_pts);
  obstacle_pts = cell2mat(reshape(padded, size(padded, 1), [], length(obstacle_pts)));
end

dim = length(lb);
A_bounds = [eye(dim);-eye(dim)];
b_bounds = [ub; -lb];

step_size = (ub - lb) ./ num_steps;

obs_regions = struct('A', cell(1, size(obstacle_pts, 3)), 'b', cell(1, size(obstacle_pts, 3)), 'C', cell(1, size(obstacle_pts, 3)), 'd', cell(1, size(obstacle_pts, 3)), 'point', cell(1, size(obstacle_pts, 3)));
for j = 1:size(obstacle_pts, 3)
  [obs_regions(j).A, obs_regions(j).b] = iris.thirdParty.polytopes.vert2lcon(obstacle_pts(:,:,j)');
end
safe_regions = struct('A', {}, 'b', {}, 'C', {}, 'd', {}, 'point', {});

for j = 1:size(initial_seeds, 2)
  seed = initial_seeds(:,j);
  [A, b, C, d] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, 'require_containment', true, 'error_on_infeasible_start', false);
  safe_regions(end+1) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', seed);
  done = callback(safe_regions(end), seed);
  if done
    return
  end
end

while length(safe_regions) < num_regions
  dists = iris.util.obstacle_distance_matrix([obs_regions, safe_regions], lb, ub, num_steps);
  [~, idx] = max(dists(:));
  subs = cell(1, dim);
  [subs{:}] = ind2sub(size(dists), idx);
  subs = cell2mat(subs)';
  seed = (subs - 1) .* step_size + lb;
  [A, b, C, d] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, seed, 'require_containment', true, 'error_on_infeasible_start', false);
  safe_regions(end+1) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', seed);
  done = callback(safe_regions(end), seed);
  if done
    return
  end
end

