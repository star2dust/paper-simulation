function test_polygon_recovery(record)
import iris.inflate_region;

if nargin < 1
  record = false;
end

r = 1;

lb = [-r;-r];
ub = [r;r];

n_obs = 7;
th = linspace(0,2*pi,n_obs+1);
% th = [0,pi/4,pi/2,pi,5*pi/4,3*pi/2,2*pi];

% th = cumsum(rand(6, 1) * 3*pi./2);
% th = th * (2*pi / th(end));

obstacles = zeros(2, 2, length(th)-1);
for j = 1:length(th)-1
  obstacles(:,:,j) = [[r*cos(th(j));r*sin(th(j))],[r*cos(th(j+1));r*sin(th(j+1))]];
end

A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-lb; ub];

start = 0.5 .* (rand(2,1) .* (ub - lb) + lb);
% start = [0;0];
% start = [0;0.98];


% profile on
[A,b,C,d,results] = inflate_region(obstacles, A_bounds, b_bounds, start);
% profile viewer
iris.drawing.animate_results(results, record);

end
