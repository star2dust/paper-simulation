function [ h ] = draw_Nd( A,b,C,d,obstacles,lb,ub )
  import iris.drawing.*;
  import iris.thirdParty.polytopes.*;

  h = figure(2);
  cla
  hold on
  for j = 1:size(obstacles, 3)
    obs = intersect_obs_with_plane(obstacles{j}, 3);
    drawPolyFromVertices(obs,'k','FaceAlpha',1);
  end
  if ~isempty(A)
    V = lcon2vert(A, b);
    drawPolyFromVertices(intersect_obs_with_plane(V', 3), 'r');
  end
  xlim([lb(1),ub(1)])
  ylim([lb(2),ub(2)])
  zlim([lb(3),ub(3)])
  camtarget(0.5*(lb(1:3)+ub(1:3)))
  campos([lb(1)-2,lb(2)-2,ub(3)])

end

