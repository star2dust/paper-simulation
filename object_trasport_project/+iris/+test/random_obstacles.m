function obstacle_pts = random_obstacles(dim, n_obs, lb, ub, scale)
% Generate a randomly distributed field of random obstacles. Each
% obstacle consists of 2^dim points uniformly randomly distributed
% within a 2*scale-length cube around a randomly selected center point
if nargin < 5
  scale = 1;
end
offsets = scale * (rand(dim, 2^dim * n_obs) * 2 - 1);
centers = bsxfun(@plus, bsxfun(@times, rand(dim, n_obs), ub - lb), lb);
centers = reshape(centers, dim, 1, []);
obstacle_pts = bsxfun(@plus, centers, reshape(offsets ./ (n_obs^(1/dim)), [dim, 2^dim, n_obs]));

end

