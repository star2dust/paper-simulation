%%
%  Copyright: Copyright (c) MOSEK ApS, Denmark. All rights reserved.
%
%  File:      lownerjohn_inner.m
%
%  Purpose:
%  Computes the Lowner-John inner ellipsoidal
%  approximation of a polytope.
%
%
%  The inner ellipsoidal approximation to a polytope
%
%     S = { x \in R^n | Ax < b }.
%
%  maximizes the volume of the inscribed ellipsoid,
%
%     { x | x = C*u + d, || u ||_2 <= 1 }.
%
%  The volume is proportional to det(C)^(1/n), so the
%  problem can be solved as
%
%    maximize         t
%    subject to       t       <= det(C)^(1/n)
%                || C*ai ||_2 <= bi - ai^T * d,  i=1,...,m
%                  C is PSD
%
%  which is equivalent to a mixed conic quadratic and semidefinite
%  programming problem.
%
%
%  References:
%  [1] "Lectures on Modern Optimization", Ben-Tal and Nemirovski, 2000.
%
function [C, d] = lownerjohn_inner(A, b)

import mosek.fusion.*;
import mosek_lownerjohn.det_rootn;

M = Model('lownerjohn_inner');

[m, n] = size(A);

% Setup variables
t = M.variable('t', 1, Domain.greaterThan(0.0));
C = M.variable('C', NDSet(n,n), Domain.unbounded());
d = M.variable('d', n, Domain.unbounded());

% (bi - ai^T*d, C*ai) \in Q
for i=1:m,
    M.constraint( sprintf('qc%d', i),  ...
                  Expr.vstack(Expr.sub(b(i) ,Expr.dot(A(i,:),d) ) , Expr.mul(C,A(i,:) )), ...
                  Domain.inQCone() );
end

% t <= det(C)^{1/n}
det_rootn(M, C, t)

% Objective: Maximize t
M.objective(ObjectiveSense.Maximize, t)

M.solve()

C = reshape(C.level(), n, n);
d = reshape(d.level(), n, 1);

M.dispose();