function test_points_2d()
import iris.drawing.*
import iris.inflate_region;

m = 10;
n = 10;
grid = rand(m, n) < 0.5;

idx = find(~grid);
[r,c] = ind2sub(size(grid), idx);
obstacles = reshape([r'; c'], [2, 1, length(r)]);

lb = [1;1];
ub = [m;n];
A = [-1,0;0,-1;1,0;0,1];
b = [-lb;ub];
start = [m/2 + 0.25; n/2 + 0.25];

[A,b,C,d, results] = inflate_region(obstacles, A, b, start);
animate_results(results);

end