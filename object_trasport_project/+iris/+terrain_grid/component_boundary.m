function boundary = component_boundary(grid, ndx)

if length(ndx) == 2
  ndx = sub2ind(size(grid), ndx(1), ndx(2));
end

filled = imfill(~grid, ndx, 8);
component = ~(xor(filled,grid));
boundary = iris.terrain_grid.find_boundary(component);

% figure(21)
% subplot(411)
% imshow(grid)
% subplot(412)
% imshow(component)
% subplot(413)
% imshow(fast_boundary);
% subplot(414)
% imshow(boundary);
% assert(all(all(fast_boundary == boundary)));
