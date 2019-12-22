function [C, d] = spot_ellipsoid(A, b)

n = size(A, 2);

% tic
pr = spotsosprog;
[pr, t] = pr.newFree(1);
[pr, Z] = pr.newFree(n, n);
[pr, C] = pr.newSym(n);
[pr, d] = pr.newFree(n);
z = diag(Z);
pr = pr.withPSD([C, Z'; Z, diag(z)]);
lor = [b - A * d, A * C']';
pr = pr.withLor(lor); 

l = ceil(log2(n));
m = 2^l - n;
if m > 0
  x = [z; repmat(t, m, 1)];
else
  x = z;
end
[pr, g] = geo_mean_recursive(pr, x);
pr = pr.withEqs(2^(l/2) * t - g);

% fprintf(1, 'setup: %f\n', toc);

% tic
% disp('mosek')
% solver = @spot_mosek;
% sol = pr.minimize(-t, solver);
% sol.eval(lor)
% double(sol.eval(C))
% double(sol.eval(d))
% fprintf(1, 'mosek: %f\n', toc);

% tic
solver = @spot_sedumi;
sol = pr.minimize(-t, solver);
% sol.eval(lor)
% fprintf(1, 'sedumi: %f\n', toc);

% tic
C = double(sol.eval(C));
d = double(sol.eval(d));
% fprintf(1, 'extract: %f\n', toc);

end

function [pr, g] = geo_mean_recursive(pr, x)
  n = length(x);
  if n > 1
    [pr, y] = pr.newFree(n/2);
    pr = pr.withRLor([reshape(x, 2, n/2); reshape(y, 1, n/2)]);
    [pr, g] = geo_mean_recursive(pr, y);
  else
    g = x;
  end
end