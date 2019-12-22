function h = draw_3d(A,b,C,d,obstacles,lb,ub)
  import iris.drawing.drawPolyFromVertices;
  import iris.thirdParty.polytopes.*;
  
  h = figure(2);
  cla
  hold on
  if ~isempty(obstacles)
    for j = 1:numel(obstacles)
      drawPolyFromVertices(obstacles{j},'k','FaceAlpha',0.5);
    end
  end
  
  if ~isempty(A)
    V = lcon2vert(A, b);
    drawPolyFromVertices(V', 'r');
  end
  th = linspace(0,2*pi,20);
  y = [cos(th);sin(th);zeros(size(th))];
  for phi = linspace(0,pi,10)
    T = makehgtform('xrotate', phi);
    R = T(1:3,1:3);
    y = [y, R * y];
  end
  x = bsxfun(@plus, C*y, d);
  drawPolyFromVertices(x, 'b', 'FaceAlpha', 1)
  xlim([lb(1),ub(1)])
  ylim([lb(2),ub(2)])
  zlim([lb(3),ub(3)])
  camtarget(0.5*(lb+ub))
  campos([lb(1)-2,lb(2)-2,ub(3)])
%   axis off;
end