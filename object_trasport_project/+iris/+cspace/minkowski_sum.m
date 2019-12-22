function p = minkowski_sum(a, b)

if size(a, 2) == 1 || size(b, 2) == 1
  p = bsxfun(@plus, a, b);
else
  p = zeros(2, size(a, 2) * size(b, 2));
  idx = 0;
  for j = 1:size(a, 2)
    p(:,idx+(1:size(b,2))) = bsxfun(@plus, a(:,j), b);
    idx = idx + size(b,2);
  end
  k = convhull(p(1,:), p(2,:), 'simplify', true);
  p = p(:,k(1:end-1));
end
