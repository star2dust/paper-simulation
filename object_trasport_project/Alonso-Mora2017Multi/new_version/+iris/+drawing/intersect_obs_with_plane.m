function [ obs ] = intersect_obs_with_plane(obs, dim)
% Find the [dim]-dimensional intersection of the N-d convex body defined by 
% vertices obs with the x[dim+1] = x[dim+2] = .... = xN = 0 surface.


while size(obs,1) > dim
  % Reduce one dimension at a time. Probably very inefficient.
  pts = [];
  N = size(obs, 1);
  for j = 1:size(obs, 2)
    for k = j+1:size(obs, 2)
        if sign(obs(end,j)) ~= sign(obs(end,k))
          d1 = abs(obs(:,j)' * [zeros(N-1,1); 1]);
          d2 = abs(obs(:,k)' * [zeros(N-1,1); 1]);
          intersect = (d2 * obs(:,j) + d1 * obs(:,k)) / (d1 + d2);
          assert(intersect(end) == 0);
          pts(:,end+1) = intersect(1:end-1);
        elseif all(obs(end,j) == 0)
          pts(:,end+1) = obs(:,j);
        elseif all(obs(end,k) == 0)
          pts(:,end+1) = obs(:,k);
        end
    end
  end
  try
    k = convhull(pts(1,:), pts(2,:), 'simplify', true);
  catch
    k = 1:size(pts,2);
  end
  obs = pts(:,k);
end

