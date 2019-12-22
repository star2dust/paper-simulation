function [A, b, infeas_start] = separating_hyperplanes(obstacle_pts, C, d)

  persistent mosek_res

  dim = size(C,1);
  infeas_start = false;
  n_obs = size(obstacle_pts, 3);
  pts_per_obs = size(obstacle_pts, 2);
  Cinv = inv(C);
  Cinv2 = (Cinv * Cinv');
  if n_obs == 0 || isempty(obstacle_pts)
    A = zeros(0, dim);
    b = zeros(0, 1);
    infeas_start = false;
    return;
  end

  uncovered_obstacles = true(n_obs,1);
  planes_to_use = false(n_obs, 1);


  image_pts = reshape(Cinv * bsxfun(@minus, reshape(obstacle_pts, dim, []), d), size(obstacle_pts));
  image_dists = reshape(sum(image_pts.^2, 1), size(obstacle_pts, 2), size(obstacle_pts, 3));
  obs_image_dists = min(image_dists, [], 1);
  [~, obs_sort_idx] = sort(obs_image_dists);

  flat_obs_pts = reshape(obstacle_pts, dim, []);

  A = zeros(n_obs,dim);
  b = zeros(n_obs,1);

  for i = obs_sort_idx
    if uncovered_obstacles(i)
      obs = obstacle_pts(:,:,i);
      ys = image_pts(:,:,i);
      dists = image_dists(:,i);
      [~,idx] = min(dists);
      xi = obs(:,idx);
      nhat = 2 * Cinv2 * (xi - d);
      nhat = nhat / norm(nhat);
      b0 = nhat' * xi;
      if all(nhat' * obs - b0 >= 0)
        % nhat is feasible, so we can skip the optimization
        A(i,:) = nhat';
        b(i) = b0;
      else
        if isempty(mosek_res)
          [~,mosek_res] = mosekopt('symbcon echo(0)');
        end
        ystar = iris.least_distance.mosek_ldp(ys, mosek_res);

        if norm(ystar) < 1e-3
          % d is inside the obstacle. So we'll just reverse nhat to try to push the
          % ellipsoid out of the obstacle.
          % warning('IRIS:EllipseCenterInObstacle', 'ellipse center is inside an obstacle.');
          infeas_start = true;
          A(i,:) = -nhat';
          b(i) = -nhat' * xi;
        else
          xstar = C*ystar + d;
          nhat = 2 * Cinv2 * (xstar - d);
          nhat = nhat / norm(nhat);
          A(i,:) = nhat;
          b(i) = nhat' * xstar;
        end
      end

      check = A(i,:) * flat_obs_pts >= b(i);
      check = reshape(check', pts_per_obs, []);
      excluded = all(check, 1);
      uncovered_obstacles(excluded) = false;

      planes_to_use(i) = true;
      uncovered_obstacles(i) = false;

      if ~any(uncovered_obstacles)
        break
      end
    end
  end
  A = A(planes_to_use,:);
  b = b(planes_to_use);
end