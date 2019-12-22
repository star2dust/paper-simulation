function v = cvxgen_ldp(Y)
[n, m] = size(Y);
if n < 3
  Y = [Y; zeros(3-n, m)];
end
if m < 8
  Y = [Y, repmat(Y(:,1), 1, (8-m))];
end
[vars, ~] = cvxgen_ldp_mex(struct('Y', Y), struct('verbose', 0));
v = vars.v(1:n);
end

