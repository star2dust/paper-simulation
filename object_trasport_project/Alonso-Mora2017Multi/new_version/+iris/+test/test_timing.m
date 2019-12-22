function [all_results] = test_timing(dim)
% NOTEST
% Benchmark IRIS as a function of number of obstacles (for WAFR 2014 paper)


lb = zeros(dim,1);
ub = 10 * ones(dim,1);
n_samples = 6;
n_trials = 10;
ns_obs = logspace(1, 6, n_samples);

A_bounds = [-diag(ones(dim, 1)); diag(ones(dim,1))];
b_bounds = [-lb; ub];
all_results = iris.inflation_results.empty();

start = 0.5 * (ub + lb);
for j = 1:n_samples
  for k = 1:n_trials
    ok = false;
    while ~ok
      n_obs = round(ns_obs(j))
      obstacle_pts = iris.test.random_cubic_obstacles(dim, n_obs, lb, ub);
      try
        [A,b,C,d,results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, start, []);
        ok = true;
        results.obstacles = [];
      catch e
        if strcmp(e.identifier, 'IRIS:InfeasibleStart')
          ok = false;
        else
          rethrow(e);
        end
      end
    end
    all_results(k,j) = results;
  end
end

