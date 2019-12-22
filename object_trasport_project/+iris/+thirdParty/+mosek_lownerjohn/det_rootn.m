%%
%  Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File:      det_rootn.m
%
%  Purpose: Models the hypograph of the n-th power of the
%  determinant of a positive definite matrix.
%
%  The convex set (a hypograph)
%
%    C = { (X, t) \in S^n_+ x R |  t <= det(X)^{1/n} },
%
%  can be modeled as the intersection of a semidefinite cone
%
%    [ X, Z; Z^T Diag(Z) ] >= 0
%
%  and a number of rotated quadratic cones and affine hyperplanes,
%
%    t <= (Z11*Z22*...*Znn)^{1/n}  (see geometric_mean).
%
%  References:
%  [1] "Lectures on Modern Optimization", Ben-Tal and Nemirovski, 2000.
%%

function [] = det_rootn(M, X, t)

import mosek.fusion.*;
import iris.thirdParty.mosek_lownerjohn.geometric_mean;

n = sqrt(X.size());

% Setup variables
Y = M.variable(Domain.inPSDCone(2*n));

% Setup Y = [X, Z; Z^T diag(Z)]
Y11 = Y.slice([1,   1],   [n+1,   n+1]);
Y21 = Y.slice([n+1, 1],   [2*n+1, n+1]);
Y22 = Y.slice([n+1, n+1], [2*n+1, 2*n+1]);

S = Matrix.sparse(n, n, 1:n, 1:n, ones(1,n));
M.constraint( Expr.sub(Expr.mulElm(S,Y21), Y22), Domain.equalsTo(0.0) );
M.constraint( Expr.sub(X, Y11), Domain.equalsTo(0.0) );

% t^n <= (Z11*Z22*...*Znn)
z = M.variable(n, Domain.unbounded());
for i=1:n,
    M.constraint( Expr.sub(z.index(i), Y22.index(i,i)), ...
                  Domain.equalsTo(0.0) );
end

geometric_mean(M, z, t);
