function x = sample_convex_polyhedron(A, b, n)
% Generate n random samples from the convex polyhedron defined
% as the intersection of half-spaces Ax <= b
import iris.thirdParty.polytopes.*;
if nargin < 3
  n = 1;
end

v = lcon2vert(A,b)';

lb = min(v, [], 2);
ub = max(v, [], 2);

dim = size(A,2);
nsamples = 0;
x = zeros(dim, n);
while nsamples < n
  z = rand(dim, 1) .* (ub - lb) + lb;
  if all(A * z <= b)
    nsamples = nsamples + 1;
    x(:,nsamples) = z;
  end
end

