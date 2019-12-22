function test_uav_demo(dim)
% This function creates an interactive demonstration of the way we can combine
% IRIS with a mixed-integer program to solve path planning problems around
% obstacles. The figure which is created shows an environment with several
% obstacles (black) and several IRIS regions which have been seeded. Each IRIS
% region is shown by a green dot at its seed point, a red polyhedron, and a blue
% ellipsoid. In addition, a trajectory for a simple "UAV" model is shown in
% magenta. The UAV is modeled as a double integrator, and we plan a trajectory
% to minimize total acceleration over a discrete trajectory. You can interact
% with the plot by dragging the seed points of the IRIS regions and by
% dragging the start or end of the UAV path.
%
% You may notice that by default, the discrete poses of the UAV are all
% outside the obstacles (or, more accurately, inside the obstacle-free IRIS
% regions) but the straight-line path between them is not. To fix this, we can
% simply lift our representation of the obstacles into four dimensions: x, y,
% xdot, and ydot and require that the 4D pose of the UAV in that space be
% outside the lifted obstacles. To try this, just set dim to '4d'.
% 
% The mixed-integer program used to find the UAV path can be seen in the
% uav_path function at the end of this file.
%
% @param dim a string, either '2d' (default) or '4d'. '2d' causes the
%            obstacles to be represented only in x, y, which has the effect of ensuring
%            that the discrete poses of the UAV trajectory are outside the obstacles,
%            but not the straight-line path between those poses. '4d' uses a four-
%            dimensional representation of the obstacles  in x, y, xdot, ydot to ensure 
%            that the straight-line path is also obstacle-free.

if nargin < 1
  dim = '2d';
end

if strcmp(dim, '2d')
  use_4d = false;
elseif strcmp(dim, '4d')
  use_4d = true;
end

dt = 1;

% The extent of the 4D position-velocity obstacles in '4d' mode. This should 
% be as large as or larger than the maximum velocity bounds in uav_path()
MAX_VEL = 0.25;

lb = [-1; -1; -MAX_VEL; -MAX_VEL];
ub = [1; 1; MAX_VEL; MAX_VEL];
if use_4d
  A_bounds = [-eye(4); eye(4)];
  b_bounds = [-lb; ub];
else
  A_bounds = [-eye(2); eye(2)];
  b_bounds = [-lb(1:2); ub(1:2)];
end

% Generate the obstacles
if use_4d
  obstacle_pts = zeros(4,12,0);
else
  obstacle_pts = zeros(2,4,0);
end
obs = [-.5, -.5, 0, 0; -.5, .5, .5, -.5];
if use_4d
  obstacle_pts = cat(3, obstacle_pts, c_obs(obs, MAX_VEL));
else
  obstacle_pts = cat(3, obstacle_pts, obs);
end
obs = [.2, .2, .4, .4; .05, 1, 1, .05];
if use_4d
  obstacle_pts = cat(3, obstacle_pts, c_obs(obs, MAX_VEL));
else
  obstacle_pts = cat(3, obstacle_pts, obs);
end
obs = [.2, .2, .4, .4; -.05, -1, -1, -.05];
if use_4d
  obstacle_pts = cat(3, obstacle_pts, c_obs(obs, MAX_VEL));
else
  obstacle_pts = cat(3, obstacle_pts, obs);
end

start = [-.75; 0];
goal = [0.7; -0.3];

% Choose the initial IRIS seed points
seeds = [[-0.75; 0.3], [-0.1; .75], [-0.1;-.75], [.75;-0.3], [0.1;0.3], [0.3; 0]];

% Build the initial IRIS regions
safe_regions = struct('A', {}, 'b', {}, 'C', {}, 'd', {}, 'point', {});
for j = 1:size(seeds, 2)
  if use_4d
    s = [seeds(:,j); 0; 0];
  else
    s = seeds(:,j);
  end
    
  [A, b, C, d, results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, s, struct('require_containment', true, 'error_on_infeas_start', true));
  safe_regions(end+1) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', s(1:2));
end

v = uav_path(start, goal, safe_regions);

h = figure(1);
handles.start = [];
handles.goal = [];
handles.region_p = [];
handles.region_e = [];
handles.region_seed = [];
handles.traj = [];
handles.obs = [];
clf
function draw(start, goal, safe_regions, obstacle_pts, v)
  clf
  hold on
  for j = 1:size(obstacle_pts, 3)
    k = convhull(obstacle_pts(1,1:4,j), obstacle_pts(2,1:4,j));
    handles.obs(j) = patch(obstacle_pts(1,k,j), obstacle_pts(2,k,j), 'k');
  end


  for j = 1:length(safe_regions)
    V = iris.thirdParty.polytopes.lcon2vert(safe_regions(j).A, safe_regions(j).b);
    V = V';
    V = V(1:2, convhull(V(1,:), V(2,:)));
    handles.region_p(j) = plot(V(1,:), V(2,:), 'Color', [.9,.3,.3], 'LineStyle', '-', 'LineWidth', 1);
    th = linspace(0,2*pi,100);
    y = [cos(th);sin(th)];
    x = bsxfun(@plus, safe_regions(j).C(1:2,1:2)*y, safe_regions(j).d(1:2));
    handles.region_e(j) = plot(x(1,:), x(2,:), 'Color', [.3,.3,.9], 'LineStyle', '-', 'LineWidth', 1);
    handles.region_seed(j) = plot(safe_regions(j).point(1), safe_regions(j).point(2), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');
  end

  handles.start = plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 15);
  handles.goal = plot(goal(1), goal(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 15);

  handles.traj = plot(v.x.value(1,:), v.x.value(2,:), 'mo-', 'LineWidth', 2);
  xlim([-1.05,1.05])
  ylim([-1.05,1.05])
end
draw(start, goal, safe_regions, obstacle_pts, v);

% 'active' indicates which element is currently being interacted with by the user
% 0: none
% 1: start pose
% 2: goal pose
% 3+: an IRIS region seed
active = 0;

  function MouseClick(src,evt)
    candidates = [start, goal, safe_regions.point];
    C = get (gca, 'CurrentPoint');
    click = [C(1,1); C(1,2)];
    [~,j] = min(sum(bsxfun(@minus, click, candidates).^2));
    active = j;
  end

  function MouseRelease(src,evt)
    active = 0;
  end

  function MouseMove(src,evt)
    C = get (gca, 'CurrentPoint');
    click = [C(1,1); C(1,2)];
    if active == 1
      start = click;
      update_handle(handles.start, start(1), start(2));
      try
        v = uav_path(start, goal, safe_regions);
        update_handle(handles.traj, v.x.value(1,:), v.x.value(2,:));
        set(handles.start, 'Marker', 'o', 'MarkerSize', 15);
      catch exception
        set(handles.start, 'Marker', 'x', 'MarkerSize', 30);
      end
    elseif active == 2
      goal = click;
      update_handle(handles.goal, goal(1), goal(2));
      try
        v = uav_path(start, goal, safe_regions);
        update_handle(handles.traj, v.x.value(1,:), v.x.value(2,:));
        set(handles.goal, 'Marker', 'o', 'MarkerSize', 15);
      catch exception
        set(handles.goal, 'Marker', 'x', 'MarkerSize', 30);
      end
    elseif active > 2
      if use_4d
        s = [click; 0;0];
      else
        s = click;
      end
      update_handle(handles.region_seed(active-2), s(1), s(2));
      try
        [A, b, C, d, results] = iris.inflate_region(obstacle_pts, A_bounds, b_bounds, s, struct('require_containment', true, 'error_on_infeas_start', true));
        safe_regions(active-2) = struct('A', A, 'b', b, 'C', C, 'd', d, 'point', s(1:2));
        V = iris.thirdParty.polytopes.lcon2vert(A, b);
        V = V';
        V = V(1:2, convhull(V(1,:), V(2,:)));
        update_handle(handles.region_p(active-2), V(1,:), V(2,:));
        th = linspace(0,2*pi,100);
        y = [cos(th);sin(th)];
        x = bsxfun(@plus, C(1:2,1:2)*y, d(1:2));
        update_handle(handles.region_e(active-2), x(1,:), x(2,:));
        set(handles.region_seed(active-2), 'Marker', 'o');
      catch exception
        set(handles.region_seed(active-2), 'Marker', 'x');
      end
    end
    drawnow();
  end
set(h, 'WindowButtonDownFcn', @MouseClick);
set(h, 'WindowButtonUpFcn', @MouseRelease);
set(h, 'WindowButtonMotionFcn', @MouseMove);
end


function obstacle_pts = c_obs(obs, max_vel)
  % Build 4d position-velocity configuration space obstacles
  obstacle_pts = zeros(4,12,0);
  for d = [1, 2]
    for s = [-1, 1]
      dx = zeros(2,4);
      dx(d,:) = s * max_vel;
      obstacle_pts = cat(3, obstacle_pts, [[obs; zeros(2,4)], [obs, obs - dx; dx, dx]]);
    end
  end
end

function p = minkowski_sum(a, b)
  p = zeros(2, size(a,2) * size(b,2));
  idx = 1;
  for j = 1:size(a,2)
    for k = 1:size(b,2)
      p(:,idx) = a(:,j) + b(:,k);
      idx = idx + 1;
    end
  end

  k = convhull(p(1,:), p(2,:));
  assert(k(1) == k(end));
  p = p(:,k(1:end-1));
end

function update_handle(h, x, y)
  set(h, 'XData', x);
  set(h, 'YData', y);
end

function v = uav_path(start, goal, safe_regions, contain_line)
  % Solve a mixed-integer quadratic program to find the path for a discrete
  % trajectory of our UAV model, such that each pose in the trajectory is
  % inside one safe region while minimizing total acceleration.

  if nargin < 4
    contain_line = false;
  end

  nv = 0;
  v = struct();

  function add_var(name, type_, size_, lb, ub, start_)
    v.(name) = struct();
    v.(name).type = type_;
    v.(name).size = size_;
    v.(name).i = reshape(nv + (1:prod(v.(name).size)), v.(name).size);
    nv = nv + v.(name).i(end);
    if isscalar(lb)
      lb = repmat(lb, v.(name).size);
    end
    if isscalar(ub)
      ub = repmat(ub, v.(name).size);
    end
    v.(name).lb = lb;
    v.(name).ub = ub;
    if nargin < 6
      start_ = [];
    end
    v.(name).start = nan(v.(name).size);
    v.(name).start(1:size(start_, 1), 1:size(start_, 2)) = start_;
  end


  n = 25;
  dt = 1;
  nr = length(safe_regions);
  add_var('x', 'C', [2, n], -1, 1);
  add_var('xd', 'C', [2, n], -.25, 0.25);
  add_var('xdd', 'C', [2, n], -0.1, 0.1);
  if contain_line
    add_var('region', 'B', [nr, n-1], 0, 1);
  else
    add_var('region', 'B', [nr, n], 0, 1);
  end
  v.x.lb(:,1) = start;
  v.x.ub(:,1) = start;
  v.xd.lb(:,1) = [0;0];
  v.xd.ub(:,1) = [0;0];
  v.xdd.lb(:,1) = [0;0];
  v.xdd.ub(:,1) = [0;0];
  v.x.lb(:,end) = goal;
  v.x.ub(:,end) = goal;
  v.xd.lb(:,end) = [0;0];
  v.xd.ub(:,end) = [0;0];
  v.xdd.lb(:,end) = [0;0];
  v.xdd.ub(:,end) = [0;0];

  A = [];
  b = [];
  Aeq = [];
  beq = [];
  Q = zeros(nv, nv);
  c = zeros(nv, 1);

  Ai = zeros(2*(n-1), nv);
  bi = zeros(size(Ai, 1), 1);
  offset = 0;
  for j = 2:n
    Ai(offset+1, v.x.i(1,j)) = 1;
    Ai(offset+1, v.xd.i(1,j-1)) = -1;
    Ai(offset+1, v.x.i(1,j-1)) = -1;
    offset = offset + 1;
    Ai(offset+1, v.x.i(2,j)) = 1;
    Ai(offset+1, v.xd.i(2,j-1)) = -1;
    Ai(offset+1, v.x.i(2,j-1)) = -1;
    offset = offset + 1;
  end
  Aeq = [Aeq; Ai];
  beq = [beq; bi];

  Ai = zeros(2*(n-1), nv);
  bi = zeros(size(Ai, 1), 1);
  offset = 0;
  for j = 2:n
    Ai(offset+1, v.xd.i(1,j)) = 1;
    Ai(offset+1, v.xdd.i(1,j-1)) = -1;
    Ai(offset+1, v.xd.i(1,j-1)) = -1;
    offset = offset + 1;
    Ai(offset+1, v.xd.i(2,j)) = 1;
    Ai(offset+1, v.xdd.i(2,j-1)) = -1;
    Ai(offset+1, v.xd.i(2,j-1)) = -1;
    offset = offset + 1;
  end
  Aeq = [Aeq; Ai];
  beq = [beq; bi];

  for j = 1:n
    Q(v.xdd.i(1,j), v.xdd.i(1,j)) = 1;
    Q(v.xdd.i(2,j), v.xdd.i(2,j)) = 1;
  end

  % Enforce membership in safe regions
  M = 100;
  if contain_line
    Ar = zeros(v.region.size(2) * sum(cellfun(@(x) 2 * size(x, 1), {safe_regions.A})), nv);
  else
    Ar = zeros(v.region.size(2) * sum(cellfun(@(x) size(x, 1), {safe_regions.A})), nv);
  end
  br = zeros(size(Ar, 1), 1);
  offset = 0;
  expected_offset = size(Ar, 1);
  for j = 1:n
    for r = 1:nr
      A_region = safe_regions(r).A;
      b_region = safe_regions(r).b;

      if size(A_region, 2) == 4
        Ai = zeros(size(A_region, 1), nv);
        bi = b_region;
        Ai(:,[v.x.i(:,j);v.xd.i(:,j)]) = A_region;
      else
        s = size(A_region, 1);
        if contain_line
          Ai = zeros(2*s, nv);
          bi = [b_region; b_region];
          Ai(1:s, v.x.i(:,j)) = A_region;
          Ai(s+(1:s), v.x.i(:,j)) = A_region;
          Ai(s+(1:s), v.xd.i(:,j)) = A_region * dt;
        else
          Ai = zeros(s, nv);
          bi = b_region;
          Ai(:,v.x.i(:,j)) = A_region;
        end
      end
        
      Ai(:,v.region.i(r,j)) = M;
      bi = bi + M;
      Ar(offset + (1:size(Ai, 1)), :) = Ai;
      br(offset + (1:size(Ai, 1)), :) = bi;
      offset = offset + size(Ai, 1);
    end
  end
  assert(offset == expected_offset);
  A = [A; Ar];
  b = [b; br];

  % Must be in a safe region
  offset = 0;
  Aeq_i = zeros(n, nv);
  beq_i = zeros(size(Aeq_i, 1), 1);
  expected_offset = size(Aeq_i, 1);
  for j = 1:v.region.size(2)
    Aeq_i(offset+1, v.region.i(:,j)) = 1;
    beq_i(offset+1) = 1;
    offset = offset + 1;
  end
  assert(offset == expected_offset);
  Aeq = [Aeq; Aeq_i];
  beq = [beq; beq_i];

  var_names = fieldnames(v);
  clear model params
  model.A = sparse([A; Aeq]);
  model.rhs = [b; beq];
  model.sense = [repmat('<', size(A,1), 1); repmat('=', size(Aeq, 1), 1)];
  model.start = nan(nv, 1);
  model.obj = c;
  model.Q = sparse(Q);

  % Set up defaults so we can fill them in from v
  model.vtype = repmat('C', nv, 1);
  model.lb = -inf(nv, 1);
  model.ub = inf(nv, 1);
  for j = 1:length(var_names)
    name = var_names{j};
    i = reshape(v.(name).i, [], 1);
    model.vtype(i) = v.(name).type;
    model.lb(i) = reshape(v.(name).lb, [], 1);
    model.ub(i) = reshape(v.(name).ub, [], 1);
    model.start(i) = reshape(v.(name).start, [], 1);
  end


  params.mipgap = 1e-4;
  params.outputflag = 0;
  % Solve the problem
  result = gurobi(model, params);

  % Extract the solution
  for j = 1:length(var_names)
    name = var_names{j};
    i = reshape(v.(name).i, [], 1);
    if v.(name).type == 'I' 
      v.(name).value = reshape(round(result.x(i)), v.(name).size);
    elseif v.(name).type == 'B'
      v.(name).value = reshape(logical(round(result.x(i))), v.(name).size);
    else
      v.(name).value = reshape(result.x(i), v.(name).size);
    end
  end

end