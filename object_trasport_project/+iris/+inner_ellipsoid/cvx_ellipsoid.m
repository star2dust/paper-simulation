function [C, d] = cvx_ellipsoid(A, b)

dim = size(A,2);

cvx_begin sdp quiet
  cvx_solver SDPT3
  variable C(dim,dim) semidefinite
  variable d(dim)
  maximize(det_rootn(C))
  subject to
    for i = 1:length(b)
      [(b(i) - A(i,:) * d) * eye(dim), C * (A(i,:)');
       (C * (A(i,:)'))', (b(i) - A(i,:) * d)] >= 0;
    end
cvx_end
end

