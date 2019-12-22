function [A, b, C, d, results] = inflate_region(obstacles, A_bounds, b_bounds, start, varargin)
import iris.*;

p = inputParser();
p.addOptional('require_containment', false, @isscalar);
p.addOptional('error_on_infeasible_start', false, @isscalar);
p.addOptional('termination_threshold', 2e-2, @(x) x > 0);
p.addOptional('iter_limit', 100, @isnumeric);
p.parse(varargin{:});
options = p.Results;

if exist('+iris/inflate_regionmex', 'file')
  if ~iscell(obstacles)
    obs_cell = cell(1, size(obstacles, 3));
    for j = 1:size(obstacles, 3)
      obs_cell{j} = obstacles(:,:,j);
    end
    obstacles = obs_cell;

    % Note: could replace this with the following, but mat2cell is 100 times slower than the for loop
    % obstacles = mat2cell(obstacles, size(obstacles, 1), size(obstacles, 2), ones(1, size(obstacles, 3)));
  end

  if nargout > 4
    [A, b, C, d, p_history, e_history] = inflate_regionmex(obstacles, A_bounds, b_bounds, start, options);
    results = inflation_results();
    results.start = start;
    results.obstacles = obstacles;
    results.n_obs = numel(obstacles);
    results.e_history = e_history;
    results.p_history = p_history;
  else
    [A, b, C, d] = inflate_regionmex(obstacles, A_bounds, b_bounds, start, options);
  end
else
  if iscell(obstacles)
    padded = pad_obstacle_points(obstacles);
    obstacle_pts = cell2mat(reshape(padded, size(padded, 1), [], length(obstacles)));
  else
    obstacle_pts = obstacles;
  end
  [A, b, C, d, results] = inflate_region_fallback(obstacle_pts, A_bounds, b_bounds, start, options);
  results.obstacles = obstacles;
  if ~iscell(results.obstacles)
    results.obstacles = mat2cell(results.obstacles, size(results.obstacles, 1), size(results.obstacles, 2), ones(1, size(results.obstacles, 3)));
  end
end
