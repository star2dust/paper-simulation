function boundary = find_boundary(component)
% Find all the cells which are false but which are 8-way adjacent to a true cell.

boundary = false(size(component));
[m, n] = size(component);
for dx = -1:1
  ind_x = max(1-dx, 1):min(m,m-dx);
  for dy = -1:1
    if dx == 0 && dy == 0
      continue
    end
    ind_y = max(1-dy, 1):min(n, n-dy);
    boundary(ind_x,ind_y) = boundary(ind_x,ind_y) | component(ind_x,ind_y) < component(ind_x+dx,ind_y+dy);
  end
end