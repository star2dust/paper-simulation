function test_boundary()

grid = logical(randi(2,100,100)-1);

for j = 1:20
  ndx = randi(100^2);

  filled = imfill(~grid, ndx, 8);
  component = ~(xor(filled,grid));
  t0 = tic();
  boundary = imdilate(component, strel('square', 3)) - component;
  t_dil = toc(t0);
  fprintf('imdilate: %f,   ', t_dil);
  t0 = tic();
  fast_boundary = iris.terrain_grid.find_boundary(component);
  t_bound = toc(t0);
  fprintf('fast: %f,   ratio: %f\n', t_bound, t_dil/t_bound);
  
  figure(23)
  subplot 211
  imshow(boundary, 'InitialMagnification', 'fit');
  subplot 212
  imshow(fast_boundary, 'InitialMagnification', 'fit');
  assert(all(all(boundary == fast_boundary)));
end
