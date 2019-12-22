function dists = obs_dist(grid)

dists = inf(size(grid));
while true
  old_dists = dists;
  dists(:,2:end) = min(dists(:,2:end), dists(:,1:end-1)+1);
  dists(:,1:end-1) = min(dists(:,1:end-1), dists(:,2:end)+1);
  dists(2:end,:) = min(dists(2:end,:), dists(1:end-1,:)+1);
  dists(1:end-1,:) = min(dists(1:end-1,:), dists(2:end,:)+1);
  dists(1:end,[1,end]) = 1;
  dists([1,end],1:end) = 1;
  dists(~grid) = 0;
  if all(all(dists == old_dists))
    break
  end
end

