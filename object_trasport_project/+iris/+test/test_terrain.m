function selected_points = test_terrain()
% NOTEST

import iris.terrain_grid.*;
import iris.inflate_region;
import iris.thirdParty.polytopes.*;
import iris.cspace.*;
import iris.drawing.*;
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'segmentation');
% load('example_feas_map.mat','Q');
load('data/example_heights.mat','heights','px2world');
load('data/example_state.mat', 'x0');

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
lb = [180;70;-pi];
ub = [245; 160; pi];
A_bounds = [-1,0,0;
            0,-1,0;
            0,0,-1;
            1,0,0;
            0,1,0;
            0,0,1];
b_bounds = [-lb;ub];

bot = world2px_2x3 * 0.9*bsxfun(@minus, [-0.0820,-0.0820, 0.1780, 0.1780;
                                         0.0624,-0.0624, 0.0624,-0.0624;
                                         0,0,0,0], [0.0480; 0; 0;]);
% obstacles = segment_grid(~grid(lb(1):ub(1),lb(2):ub(2)));
% for j = 1:length(obstacles)
%   obstacles{j} = bsxfun(@plus, obstacles{j}, lb(1:2)-1);
% end
% obstacles = cspace3(obstacles, bot, 4);
selected_points = [];

while true
  figure(6)
  imshow(grid, 'InitialMagnification', 'fit')
  [c,r] = ginput(1);
  if ~isempty(c)
    selected_points(:,end+1) = [r;c];
  end
  if isempty(c)
    break
  end
  c = round(c);
  r = round(r);
  black_edges = [];
  [black_edges(1,:), black_edges(2,:)] = ind2sub(size(grid), find(component_boundary(grid, [r;c])));
  obs_mask = all(bsxfun(@minus, A_bounds([1,2,4,5],1:2) * black_edges, b_bounds([1,2,4,5])) <= max(max(bot)));
  obstacles = mat2cell(black_edges(:,obs_mask) , 2, ones(1,sum(obs_mask)));
  obstacles = cspace3(obstacles, bot, 4);


  [A,b,C,d,results] = inflate_region(obstacles, A_bounds, b_bounds, [r;c;0]);
%   animate_results(results);

  [inner_poly, outer_poly] = project_c_space_region(A,b);

  % px_xy = [V(:,2)'; V(:,1)'];
  if ~isempty(inner_poly)
    px_xy = [inner_poly(2,:); inner_poly(1,:)];
    world_xy = px2world_2x3 * [px_xy; ones(1,size(px_xy,2))];
    z = ones(size(px_xy,2)) * heights(r,c) + 0.03;
    world_xyz = [world_xy; z];
    lcmgl.glColor3f(0,1,0);
    lcmgl.glLineWidth(10);
    lcmgl.glBegin(lcmgl.LCMGL_LINES);
    for j = 1:size(world_xy,2)-1
      lcmgl.glVertex3d(world_xyz(1,j),world_xyz(2,j),world_xyz(3,j));
      lcmgl.glVertex3d(world_xyz(1,j+1),world_xyz(2,j+1),world_xyz(3,j+1));
    end
    lcmgl.glVertex3d(world_xyz(1,end),world_xyz(2,end),world_xyz(3,end));
    lcmgl.glVertex3d(world_xyz(1,1),world_xyz(2,1),world_xyz(3,1));
    lcmgl.glEnd();
  end

  px_xy = [outer_poly(2,:); outer_poly(1,:)];
  world_xy = px2world_2x3 * [px_xy; ones(1,size(px_xy,2))];
  z = ones(size(px_xy,2)) * heights(r,c) + 0.03;
  world_xyz = [world_xy; z];
  lcmgl.glColor3f(1,1,0);
  lcmgl.glLineWidth(10);
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