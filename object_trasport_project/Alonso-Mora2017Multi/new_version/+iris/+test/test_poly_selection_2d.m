function test_poly_selection_2d()
% NOTEST

import iris.terrain_grid.*;
import iris.inflate_region;
import iris.thirdParty.polytopes.*;
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'segmentation');
% load('example_feas_map.mat','Q');
load('data/example_heights.mat','heights','px2world');
px2world(1,end) = px2world(1,end) - sum(px2world(1,1:3)); % stupid matlab 1-indexing...
px2world(2,end) = px2world(2,end) - sum(px2world(2,1:3));
resample = 2;
mag = 2^(resample-1);
heights = interp2(heights, (resample-1));
px2world = px2world * [1/mag 0 0 (1-1/mag); 0 1/mag 0 (1-1/mag ); 0 0 1 0; 0 0 0 1];

world2px = inv(px2world);
world2px_2x3 = world2px(1:2,[1,2,4]);
px2world_2x3 = px2world(1:2, [1,2,4]);

Q = imfilter(heights, [1, -1]) - (0.06/mag) > 0;
Q = Q | imfilter(heights, [-1, 1]) - (0.06/mag) > 0;
Q = Q | imfilter(heights, [1; -1]) - (0.06/mag) > 0;
Q = Q | imfilter(heights, [-1; 1]) - (0.06/mag) > 0;
Q(isnan(heights)) = 1;
% grid = Q(85:125,25:85);
grid = ~Q;
[all_squares(1,:), all_squares(2,:)] = ind2sub(size(grid), 1:length(reshape(grid,[],1)));

while true
  figure(2)
  subplot(211)
  imshow(grid, 'InitialMagnification', 'fit')
  [c,r] = ginput(1);
  if isempty(c)
    break
  end
  c = round(c);
  r = round(r);
  clear white_squares black_squares black_edges
  [white_squares(1,:), white_squares(2,:)] = ind2sub(size(grid), find(grid));
  [black_squares(1,:), black_squares(2,:)] = ind2sub(size(grid), find(~grid));
  black_edges = [];
  [black_edges(1,:), black_edges(2,:)] = ind2sub(size(grid), find(component_boundary(grid, [r;c])));

  obstacles = mat2cell(black_edges, 2, ones(1,size(black_edges,2)));

  lb = [0;0];
  ub = [size(grid,1); size(grid,2)];
  A_bounds = [-1,0;0,-1;1,0;0,1];
  b_bounds = [-lb; ub];

  [A,b,C,d] = inflate_region(obstacles, A_bounds, b_bounds, [r;c]);

  figure(2)
  subplot(212)
  cla
  hold on
  for j = 1:length(obstacles)
    patch(obstacles{j}(1,:), obstacles{j}(2,:), 'k');
  end
  V = lcon2vert(A, b);
  k = convhull(V(:,1), V(:,2));
  plot(V(k,1), V(k,2), 'ro-');
  th = linspace(0,2*pi,100);
  y = [cos(th);sin(th)];
  x = bsxfun(@plus, C*y, d);
  plot(x(1,:), x(2,:), 'b-');
  xlim([lb(1),ub(1)])
  ylim([lb(1),ub(2)])
%   grid(all(bsxfun(@le, A * all_squares, b), 1)) = 0;
  
  V = V(k,:);
  px_xy = [V(:,2)'; V(:,1)'];
  world_xy = px2world_2x3 * [px_xy; ones(1,size(px_xy,2))];
  z = ones(size(px_xy,2)) * heights(r,c) + 0.03;
  world_xyz = [world_xy; z];
  lcmgl.glColor3f(1,0,0);
  lcmgl.glLineWidth(10);
  lcmgl.glBegin(lcmgl.LCMGL_LINES);
  for j = 1:size(world_xy,2)-1
    lcmgl.glVertex3d(world_xyz(1,j),world_xyz(2,j),world_xyz(3,j));
    lcmgl.glVertex3d(world_xyz(1,j+1),world_xyz(2,j+1),world_xyz(3,j+1));
  end
  lcmgl.glVertex3d(world_xyz(1,end),world_xyz(2,end),world_xyz(3,end));
  lcmgl.glVertex3d(world_xyz(1,1),world_xyz(2,1),world_xyz(3,1));
  lcmgl.glEnd();
  
  px_xy = [x(2,:); x(1,:)];
  world_xy = px2world_2x3 * [px_xy; ones(1,size(px_xy,2))];
  z = ones(size(px_xy,2)) * heights(r,c) + 0.03;
  world_xyz = [world_xy; z];
  lcmgl.glColor3f(0,0,1);
  lcmgl.glLineWidth(5);
  lcmgl.glBegin(lcmgl.LCMGL_LINES);
  for j = 1:size(world_xy,2)-1
    lcmgl.glVertex3d(world_xyz(1,j),world_xyz(2,j),world_xyz(3,j));
    lcmgl.glVertex3d(world_xyz(1,j+1),world_xyz(2,j+1),world_xyz(3,j+1));
  end
  lcmgl.glVertex3d(world_xyz(1,end),world_xyz(2,end),world_xyz(3,end));
  lcmgl.glVertex3d(world_xyz(1,1),world_xyz(2,1),world_xyz(3,1));
  lcmgl.glEnd();
  
  lcmgl.glColor3f(0,1,0);
  lcmgl.sphere([(px2world_2x3 * [c;r;1])', heights(r,c)], 0.05, 20, 20);
  
  
%   lcmgl.switchBuffers();
%   plot_lcm_points([px_xy', px_z'],repmat([1,0,0],length(px_z),1),101,'region1',3,1);
end
lcmgl.switchBuffers();
end