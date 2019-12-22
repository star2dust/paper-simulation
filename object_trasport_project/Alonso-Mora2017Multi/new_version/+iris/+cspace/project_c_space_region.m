function [inner_poly, outer_poly] = project_c_space_region(A, b )
% Given a polyhedron of free C-space Ax <= b, project it down into the plane.
% inner_poly is a patch of xy poses where the polyhedron contains all orientations of the
% bot, and outer_poly is a patch of xy poses where the polyhedron contains only some
% orientations of the bot. inner_poly may be empty.

import iris.thirdParty.polytopes.lcon2vert;
% figure(h)
V = lcon2vert(A, b)';

outer_poly = V(1:2,convhull(V(1,:), V(2,:), 'simplify', true));
% patch(projected_outer_poly(1,:), projected_outer_poly(2,:), 'y', 'FaceAlpha', 0.5);

top_face = [];
bottom_face = [];

top_face_pts = V(:,abs(V(3,:) - pi) < 1e-4);
if size(top_face_pts, 2) > 2
  try
    k = convhull(top_face_pts(1,:), top_face_pts(2,:), 'simplify', true);
    top_face = top_face_pts(:,k(end:-1:1));
  end
end

bottom_face_pts = V(:,abs(V(3,:) + pi) < 1e-4);
if size(bottom_face_pts, 2) > 2
  try
    k = convhull(bottom_face_pts(1,:), bottom_face_pts(2,:), 'simplify', true);
    bottom_face = bottom_face_pts(:,k(end:-1:1));
  end
end

inner_poly = [];
if ~isempty(top_face) && ~isempty(bottom_face)
  [inter_x, inter_y] = polybool('intersection',...
                            top_face(1,:), top_face(2,:),...
                            bottom_face(1,:), bottom_face(2,:));
  if ~isempty(inter_x)
    inner_poly(1,:) = inter_x;
    inner_poly(2,:) = inter_y;
  end
end

end

