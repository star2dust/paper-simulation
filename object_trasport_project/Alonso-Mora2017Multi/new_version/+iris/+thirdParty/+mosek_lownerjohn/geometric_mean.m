%%
%  Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File:      geometric_mean.py
%
%  Purpose: Models the convex set
%
%    S = { (x, t) \in R^n x R | x >= 0, t <= (x1 * x2 * ... *xn)^(1/n) }.
%
%  as the intersection of rotated quadratic cones and affine hyperplanes,
%  see [1, p. 105].  This set can be interpreted as the hypograph of the
%  geometric mean of x.
%
%  We illustrate the modeling procedure using the following example.
%  Suppose we have
%
%     t <= (x1 * x2 * x3)^(1/3)
%
%  for some t >= 0, x >= 0. We rewrite it as
%
%     t^4 <= x1 * x2 * x3 * x4,   x4 = t
%
%  which is equivalent to (see [1])
%
%     x11^2 <= 2*x1*x2,   x12^2 <= 2*x3*x4,
%
%     x21^2 <= 2*x11*x12,
%
%     sqrt(4)*x21 = t, x4 = t.
%
%  References:
%  [1] "Lectures on Modern Optimization", Ben-Tal and Nemirovski, 2000.
%%

function [] = geometric_mean(M, x, t)
%    Models the convex set
%
%      S = { (x, t) \in R^n x R | x >= 0, t <= (x1 * x2 * ... * xn)^(1/n) }
%
%    as the intersection of rotated quadratic cones and affine hyperplanes.

import mosek.fusion.*;

n = x.size();
l = ceil(log2(n));
m = 2^l - n;

if m>0
  x0 = Variable.vstack( x, M.variable(m, Domain.greaterThan(0.0) ) );
else
     x0=x;
end

z = x0;

for i=1:l-1,
    xi = M.variable(2^(l-i), Domain.greaterThan(0.0));

    for k=1:2^(l-i),
        M.constraint(Variable.vstack( z.index(2*k-1),z.index(2*k),xi.index(k)),...
                     Domain.inRotatedQCone());
    end
    z = xi;
end

t0 = M.variable(1, Domain.greaterThan(0.0));
M.constraint(     Variable.vstack(z, t0), Domain.inRotatedQCone());

M.constraint(Expr.sub(Expr.mul(2^(0.5*l),t),t0), Domain.equalsTo(0.0));
for i=2^l-m+1:2^l
    M.constraint(Expr.sub(x0.index(i), t), Domain.equalsTo(0.0));
end
