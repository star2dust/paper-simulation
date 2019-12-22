function [c] = cspace3(obs, bot, theta_steps, post_rotation)
if nargin < 4
  post_rotation = eye(2);
end
import iris.cspace.minkowski_sum;
if iscell(obs)
  padded = iris.pad_obstacle_points(obs);
  obs = cell2mat(reshape(padded, size(padded, 1), [], length(obs)));
end
bot = -bot;

if isscalar(theta_steps)
  th = linspace(-pi, pi, theta_steps);
else
  th = theta_steps;
end

c = zeros(3, 2*size(bot, 2), size(obs,3) * (length(th)-1));

if size(obs, 2) == 1
  idx = 0;
  for j = 1:(length(th)-1)
    rbot0 = post_rotation * iris.util.rotmat(th(j)) * bot;
    c_obs0 = bsxfun(@plus, rbot0, obs);

    rbot1 = post_rotation * iris.util.rotmat(th(j+1)) * bot;
    c_obs1 = bsxfun(@plus, rbot1, obs);

    c(:,:,idx+(1:size(obs,3))) = [c_obs0, c_obs1;
                                  th(j)-1e-3+zeros([1,size(c_obs0,2),size(obs,3)]), th(j+1)+1e-3+zeros([1,size(c_obs1,2),size(obs,3)])];
    idx = idx + size(obs,3);
  end
else
  idx = 1;
  for j = 1:(length(th)-1)
    for k = 1:size(obs,3)
      rbot0 = post_rotation * iris.util.rotmat(th(j)) * bot;
      c_obs0 = minkowski_sum(rbot0, obs(:,:,k));

      rbot1 = post_rotation * iris.util.rotmat(th(j+1)) * bot;
      c_obs1 = minkowski_sum(rbot1, obs(:,:,k));
      c_pts = [c_obs0, c_obs1;
              th(j)*ones(1,size(c_obs0,2)) - 1e-3, th(j+1)*ones(1,size(c_obs1,2)) + 1e-3];
      if size(c_pts, 2) > size(c, 2)
        c = [c, zeros([size(c, 1), size(c_pts,2)-size(c,2), size(c,3)])];
      end
      c(:,:,idx) = c_pts;
      idx = idx + 1;
    end
  end
end