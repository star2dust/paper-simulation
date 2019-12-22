function test_inner_ellipsoid()
% Test the mosek inner ellipsoid function

%% Try in 2D
m = randi(10)+3; % number of points

x = rand(m, 2); % random points of chosen size

[P, c] = iris.thirdParty.mosek_lownerjohn.lownerjohn_ellipsoid.lownerjohn_outer(x);

% P*x - c <= 1
% y = P*x - c
% Pi*y = x - Pi*c
% x = Pi*y + Pi*c

C = inv(P);
d = inv(P) * c;

th = linspace(0,2*pi,100);
y = [cos(th);sin(th)];
z = bsxfun(@plus, C*y, d);
figure(1);
clf
hold on
plot(z(1,:), z(2,:), 'b-', 'LineWidth', 2);
plot(x(:,1), x(:,2), 'ro')

for j = 1:size(x, 1)
  assert(norm(P*(x(j,:)') - c) <= 1 + 1e-3);
end


%% Try again in 3d
m = randi(10)+4; % number of points

x = rand(m, 3); % random points of chosen size

[P, c] = iris.thirdParty.mosek_lownerjohn.lownerjohn_ellipsoid.lownerjohn_outer(x);

% P*x - c <= 1
% y = P*x - c
% Pi*y = x - Pi*c
% x = Pi*y + Pi*c

C = inv(P);
d = inv(P) * c;

figure(2);
clf
hold on
th = linspace(0,2*pi,20);
y = [cos(th);sin(th);zeros(size(th))];
for phi = linspace(0,pi,10)
  T = makehgtform('xrotate', phi);
  R = T(1:3,1:3);
  y = [y, R * y];
end
z = bsxfun(@plus, C*y, d);
iris.drawing.drawPolyFromVertices(z, 'b', 'FaceAlpha', 0.5)
plot3(x(:,1), x(:,2), x(:,3), 'ro')

for j = 1:size(x, 1)
  assert(norm(P*(x(j,:)') - c) <= 1 + 1e-3);
end


