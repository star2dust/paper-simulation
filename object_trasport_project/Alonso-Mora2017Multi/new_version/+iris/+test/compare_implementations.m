function compare_implementations()

n_trials = 50;
n_obstacles = 5;
matlab_times = nan(1, n_trials);
cpp_times = nan(1, n_trials);

iter = 1;
while iter <= n_trials
  dim = 1 + ceil(iter / 20);
  A_bounds = [eye(dim); -eye(dim)];
  b_bounds = [ones(dim, 1); zeros(dim, 1)];

  obstacle_pts = iris.test.random_obstacles(dim, n_obstacles, zeros(dim,1), ones(dim,1), 0.05);
  obstacles = mat2cell(obstacle_pts, size(obstacle_pts, 1), size(obstacle_pts, 2), ones(1, size(obstacle_pts, 3)));
  start = 0.5 * ones(dim,1);
  options = struct('require_containment', false,...
                   'error_on_infeasible_start', false,...
                   'termination_threshold', 2e-2,...
                   'iter_limit', 100);

  try
    t0 = tic();
    [A_c, b_c, C_c, d_c] = iris.inflate_regionmex(obstacles, A_bounds, b_bounds, start, options);
    cpp_times(iter) = toc(t0);

    t0 = tic();
    [A_m, b_m, C_m, d_m] = iris.inflate_region_fallback(obstacle_pts, A_bounds, b_bounds, start, options);
    matlab_times(iter) = toc(t0);

    iter = iter + 1
  catch
    continue
  end

  % assert(iris.util.equal_up_to_permutations(A_c, A_m, 1e-2));
  % assert(iris.util.equal_up_to_permutations(b_c, b_m, 1e-2));
  assert(all(all(abs(C_c - C_m) < 1e-2)));
  assert(all(all(abs(d_c - d_m) < 1e-2)));

end

fprintf(1, 'c++ mean: %f std: %f\n', mean(cpp_times), std(cpp_times));
fprintf(1, 'matlab mean: %f std: %f\n', mean(matlab_times), std(matlab_times));
