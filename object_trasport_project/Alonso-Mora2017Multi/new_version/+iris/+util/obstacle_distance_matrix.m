function dists = obstacle_distance_matrix(regions, lb, ub, num_cells)

dim = length(lb);
if size(num_cells) == 1
  num_cells = repmat(num_cells, size(lb));
end
assert(all(size(lb) == size(ub)));

for j = 1:length(regions)
  for k = 1:size(regions(j).A, 1)
    n = norm(regions(j).A(k,:));
    regions(j).A(k,:) = regions(j).A(k,:) / n;
    regions(j).b(k) = regions(j).b(k) / n;
    % [regions(j).C, regions(j).d] = iris.maximal_ellipse(regions(j).A, regions(j).b);
    % regions(j).Cinv = inv(regions(j).C);
  end
end


dists = inf(reshape(num_cells, 1, []));
step_size = (ub - lb) ./ num_cells;

s = cell(1, dim);
for j = 1:numel(dists)
  [s{:}] = ind2sub(size(dists), j);
  
  x_vect = reshape(cell2mat(s), [], 1);
  x_vect = (x_vect - 1) .* step_size + lb;
  % for i = 1:size(obstacle_pts, 3)
  %   Y = bsxfun(@minus, obstacle_pts(:,:,i), x_vect);
  %   v = iris.least_distance.cvxgen_ldp(Y);
  %   dists(j) = min(dists(j), norm(v));
  % end
  dists(j) = min(dists(j), min(ub - x_vect));
  dists(j) = min(dists(j), min(x_vect - lb));
  for i = 1:length(regions)
    if all(regions(i).A * x_vect <= regions(i).b)
      dists(j) = 0;
    else
      dists(j) = min(dists(j), max(regions(i).A * x_vect - regions(i).b));
    end  
    % dists(j) = min(dists(j), norm(regions(i).Cinv * (x_vect - regions(i).d))-1);
  end

end


