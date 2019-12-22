function ystar = mosek_ldp(ys, res)
% Use Mosek to find the closest point in the convex hull of the ys to the
% origin.

if nargin < 2
  [~, res] = mosekopt('symbcon echo(0)');
end

dim = size(ys, 1);

nw = size(ys,2);
nvar= 1 + dim+nw;
prob.c   = [zeros(1, dim+nw) , 1];
prob.a   = sparse([ [-eye(dim), ys, zeros(dim,1)];[ zeros(1,dim), ones(1,nw),0] ]);
prob.blc = [zeros(dim,1);1];
prob.buc = [zeros(dim,1);1];
prob.blx = [-inf*ones(dim,1);zeros(nw+1,1)];
prob.bux = inf*ones(nvar,1);

% Specify the cones.
prob.cones.type   = res.symbcon.MSK_CT_QUAD;
prob.cones.sub    = [nvar, 1:dim];
prob.cones.subptr = 1;

% Optimize the problem.
[~,solution]=mosekopt('minimize echo(0)',prob);
%toc
ystar = solution.sol.itr.xx(1:dim);

end

