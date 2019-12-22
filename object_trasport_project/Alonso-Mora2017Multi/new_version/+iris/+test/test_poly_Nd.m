function [ results ] = test_poly_Nd(N, record)
% Demonstrate that we can apply the algorithm in arbitrary dimension

import iris.inflate_region;
import iris.drawing.*;

if nargin < 2
  record = false;
end

if nargin < 1
  N = 4;
end

lb = -ones(N,1);
ub = ones(N,1);
dim = N;
n_obs = 50;

obstacles = zeros(dim, 2^dim, n_obs);
for j = 1:n_obs
  center = rand(dim, 1) .* (ub(1) - lb(1)) + lb(1);
  offsets = rand(dim, 2^dim) .* (0.3 - (-0.3)) + (-0.3);
  obstacles(:,:,j) = bsxfun(@plus, center, offsets);
end

A_bounds = [];
for j = 1:dim
  row = zeros(1,dim);
  row(j) = -1;
  A_bounds(j,:) = row;
  b_bounds(j) = -lb(j);
end
for j = 1:dim
  row = zeros(1,dim);
  row(j) = 1;
  A_bounds(end+1,:) = row;
  b_bounds(end+1) = ub(j);
end
b_bounds = reshape(b_bounds,[],1);

start = 0.5 * (ub + lb);

% profile on
[A,b,C,d,results] = inflate_region(obstacles, A_bounds, b_bounds, start);
% profile viewer

animate_results(results, record);


end

