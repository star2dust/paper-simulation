function [obstacles] = segment_grid(grid)
import iris.terrain_grid.*
obstacles = {};
while any(any(grid))
  [mask, A, b, obs] = inflate_grid_region([],grid);
  grid(mask) = 0;
%   sum(sum(grid))
  obstacles{end+1} = obs;
end

