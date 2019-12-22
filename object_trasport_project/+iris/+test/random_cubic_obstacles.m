function obstacle_pts = random_cubic_obstacles(dim, n_obs, lb, ub, scale)
% Generate a randomly distributed field of cubic obstacles
if nargin < 5
  scale = 1;
end
if dim == 2
  offsets = scale * [1, 1, -1, -1;
                     -1, 1, 1, -1];
elseif dim == 3
  offsets = scale * [1, 1, -1, -1, 1, 1, -1, -1;
                   -1, 1, 1, -1, -1, 1, 1, -1;
                   -1,-1,-1,-1, 1, 1, 1, 1];
end
centers = bsxfun(@plus, bsxfun(@times, rand(dim, n_obs), ub - lb), lb);
centers = reshape(centers, dim, 1, []);
obstacle_pts = bsxfun(@plus, centers, repmat(offsets ./ (n_obs^(1/dim)), [1,1, n_obs]));

end
