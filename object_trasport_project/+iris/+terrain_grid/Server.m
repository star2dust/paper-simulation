classdef Server < handle
  properties(SetAccess=private, GetAccess=private)
    heightmaps=containers.Map('KeyType', 'uint32', 'ValueType', 'any');
    map_id_last_used=[];
    map_id_last_added=[];
    max_stored_maps=5;
  end

  methods
    function obj = Server()
    end

    function region = getCSpaceRegionAtIndex(obj, i0, yaw, collision_model, varargin)
      p = inputParser();
      p.addRequired('i0', @isnumeric);
      p.addRequired('yaw', @isnumeric);
      p.addRequired('collision_model', @(x) isa(x, 'iris.terrain_grid.CollisionModel'));
      p.addParamValue('map_id', -1, @isnumeric);
      p.addParamValue('xy_bounds', iris.Polyhedron(zeros(0,2),zeros(0,1)), @(x) isa(x, 'iris.Polyhedron'));
      p.addParamValue('plane_distance_tolerance', 0.025, @isnumeric);
      p.addParamValue('plane_angle_tolerance', 10 * pi/180, @isnumeric);
      p.addParamValue('excluded_grid', []);
      p.addParamValue('debug', false);
      p.addParamValue('error_on_infeasible_start', true);
      p.parse(i0, yaw, collision_model, varargin{:});
      options = p.Results;

      if options.map_id == -1
        heightmap = obj.getHeightmap(obj.map_id_last_added(end));
      else
        heightmap = obj.getHeightmap(options.map_id);
      end

      x0 = heightmap.X(i0);
      y0 = heightmap.Y(i0);
      z0 = heightmap.Z(i0);
      p0 = [x0;y0;z0];
      n0 = heightmap.normals(:,i0);

      sz = size(heightmap.X);


      dist_to_plane = abs(n0'*[reshape(heightmap.X, 1, []); reshape(heightmap.Y, 1, []); reshape(heightmap.Z, 1, [])] - n0'*p0);
      dist_to_plane = reshape(dist_to_plane, sz);
      dist_mask = abs(dist_to_plane < options.plane_distance_tolerance);

      normal_product = n0' * heightmap.normals;
      normal_product = reshape(normal_product, sz);
      normal_angle_mask = normal_product > cos(options.plane_angle_tolerance);

      plane_mask = dist_mask & normal_angle_mask;
      if ~isempty(options.excluded_grid)
        if ~options.excluded_grid(i0)
          error('IRIS:TerrainSegmentation:BadSeed', 'the seed point is marked as infeasible in the excluded_grid option');
        end
        plane_mask = plane_mask & options.excluded_grid;
      end

      if ~plane_mask(i0)
        error('IRIS:TerrainSegmentation:SelectedPointInfeasible', 'cannot create a region around this point.');
      end

      boundary_mask = logical(iris.terrain_grid.component_boundary(plane_mask, i0));

      if options.debug
        figure(4)
        clf
        subplot(321)
        imshow(dist_mask, 'InitialMagnification', 'fit');
        title('dist')
        subplot(323)
        imshow(normal_angle_mask, 'InitialMagnification', 'fit');
        title('normal');
        subplot(325)
        imshow(options.excluded_grid, 'InitialMagnification', 'fit');
        title('existing');
        subplot(322)
        imshow(plane_mask, 'InitialMagnification', 'fit');
        title('combined');
        subplot(324)
        imshow(~boundary_mask, 'InitialMagnification', 'fit');
        title('boundary');
      end

      obs_x = heightmap.X(boundary_mask);
      obs_y = heightmap.Y(boundary_mask);
      obstacle_pts = reshape([reshape(obs_x, 1, []); reshape(obs_y, 1, [])], 2, 1, []);
      theta_steps = yaw + (-pi:pi/4:pi);

      %% Get obstacles for the foot shape
      % Reorient the foot onto the terrain plane
      c = cross([0;0;1], n0);
      if norm(c) >= 1e-6
        ax = c / norm(c);
        angle = asin(norm(c));
        Rplane = axis2rotmat([ax; angle]);
      else
        Rplane = eye(3);
      end

      c_obs = iris.cspace.cspace3(obstacle_pts, collision_model.foot_shape, theta_steps, Rplane(1:2,1:2));


      %% Get obstacles for the upper body collision model
      % compute distance from the heightmap Z to the current plane
      % n' * [x;y;z] = n' * point
      % z = (n'*point - n(1:2)'*[x;y])/n(3)
      dZ = heightmap.Z - reshape((n0'*p0 - n0(1:2)'*[reshape(heightmap.X,1,[]);reshape(heightmap.Y,1,[])])/n0(3), sz);
      for j = 1:length(collision_model.body_slices.z)
        zmin = collision_model.body_slices.z(j);
        if j < length(collision_model.body_slices.z)
          zmax = collision_model.body_slices.z(j+1);
        else
          zmax = inf;
        end
        z_range_mask = dZ >= zmin & dZ <= zmax;
        boundary_mask = logical(iris.terrain_grid.component_boundary(~z_range_mask, i0));
        obs_x = heightmap.X(boundary_mask);
        obs_y = heightmap.Y(boundary_mask);
        if ~isempty(obs_x)
          obstacle_pts = reshape([reshape(obs_x, 1, []); reshape(obs_y, 1, [])], 2, 1, []);
          c_obs = cat(3, c_obs, iris.cspace.cspace3(obstacle_pts, collision_model.body_slices.xy(:,:,j), theta_steps ));
        end
      end

      %% Add a bounding polyhedron
      bounds = iris.Polyhedron.from2DVertices([heightmap.X(end,1), heightmap.X(end,end), heightmap.X(1,end), heightmap.X(1,1); 
                                             heightmap.Y(end,1), heightmap.Y(end,end), heightmap.Y(1,end), heightmap.Y(1,1)]);
      % Add bounds on yaw angle
      bounds.A = [bounds.A, zeros(size(bounds.A, 1), 1); 
                  zeros(2, size(bounds.A, 2)), [-1; 1]];
      bounds.b = [bounds.b; -theta_steps(1); theta_steps(end)];
      if size(options.xy_bounds.A, 2) == 2
        options.xy_bounds.A(:,end+1:3) = 0;
      end
      bounds.A = [bounds.A; options.xy_bounds.A];
      bounds.b = [bounds.b; options.xy_bounds.b];
      [A, b, C, d] = iris.inflate_region(c_obs, bounds.A, bounds.b, [x0; y0; yaw], 'require_containment', true, 'error_on_infeasible_start', options.error_on_infeasible_start);

      [A, iA] = unique(A, 'rows');
      b = b(iA);
      region = iris.TerrainRegion(A, b, C, d, p0, n0);

      if options.debug
        figure(12)
        clf
        iris.drawing.drawPolyFromVertices(iris.thirdParty.polytopes.lcon2vert(A, b)', 'r');
      end
      
    end

    function regions = findSafeTerrainRegions(obj, map_id, collision_model, varargin)
      p = inputParser();
      p.KeepUnmatched = true;
      p.addRequired('map_id', @isnumeric);
      p.addRequired('collision_model', @(x) isa(x, 'iris.terrain_grid.CollisionModel'));
      p.addParamValue('seeds', []);
      p.addParamValue('default_yaw', 0, @isnumeric);
      p.addParamValue('max_slope_angle', 40 * pi/180, @isnumeric);
      p.addParamValue('max_height_variation', 0.05, @isnumeric);
      p.addParamValue('xy_bounds', iris.Polyhedron(zeros(0,2),zeros(0,1)), @(x) isa(x, 'iris.Polyhedron'));
      p.addParamValue('max_num_regions', inf, @(x) x > 0);
      p.addParamValue('debug', false);
      p.parse(map_id, collision_model, varargin{:});
      options = p.Results;

      region_options = p.Unmatched;
      region_options.map_id = map_id;
      region_options.xy_bounds = options.xy_bounds;
      region_options.debug = options.debug;

      regions = iris.TerrainRegion.empty();

      foot_length = max(collision_model.foot_shape(1,:)) - min(collision_model.foot_shape(1,:));

      heightmap = obj.getHeightmap(map_id);
      slope_angles = heightmap.getSlopeAngles();
      potential_safe_grid = slope_angles < options.max_slope_angle;
      sz = size(potential_safe_grid);
      
      within_bounds = all(bsxfun(@le, options.xy_bounds.A * [reshape(heightmap.X, 1, []); reshape(heightmap.Y, 1, [])], options.xy_bounds.b), 1);
      potential_safe_grid = potential_safe_grid & reshape(within_bounds,sz);
      for dx = -1:1
        for dy = -1:1
          dZ = heightmap.Z(2:end-1,2:end-1) - heightmap.Z((2+dx):(end-1+dx),(2+dy):(end-1+dy));
          potential_safe_grid(2:end-1,2:end-1) = potential_safe_grid(2:end-1,2:end-1) & abs(dZ) < options.max_height_variation;
        end
      end

      excluded_grid = true(sz);
      seed_ind = 1;

      while length(regions) < options.max_num_regions

        if options.debug
          figure(3)
          clf
          imshow(potential_safe_grid, 'InitialMagnification', 'fit');
        end

        if seed_ind <= size(options.seeds, 2)
          seed = options.seeds([1,2,6], seed_ind);
          yaw = seed(3);
          seed_ind = seed_ind + 1;
          i0 = obj.xy2ind(map_id, seed(1:2));
          % seed
          % heightmap.X(i0)
          % heightmap.Y(i0)
          % disp('here')
        else
          obs_dists = iris.terrain_grid.obs_dist(potential_safe_grid);
          [max_dist, i0] = max(obs_dists(:));
          yaw = options.default_yaw;
          if max_dist < 0.5 * foot_length / mean(heightmap.resolution);
            break;
          end
        end

        try
          region = obj.getCSpaceRegionAtIndex(i0, yaw, collision_model, region_options, 'excluded_grid', excluded_grid, 'error_on_infeasible_start', true);
        catch e
          if strcmp(e.identifier, 'IRIS:InfeasibleStart')
            excluded_grid(i0) = false;
            potential_safe_grid(i0) = false;
            continue
          else
            rethrow(e);
          end
        end
        regions(end+1) = region;
        % excluded_grid(i0) = false;
        potential_safe_grid(i0) = false;

        inpoly = all(bsxfun(@minus, region.A * [reshape(heightmap.X, 1, []); reshape(heightmap.Y, 1, []); yaw + zeros(1, numel(heightmap.X))], region.b) <= foot_length / 2, 1);
        inpoly = reshape(inpoly, sz);
        potential_safe_grid = potential_safe_grid & ~inpoly;
      end
    end

    function i0 = xy2ind(obj, map_id, xy)
      heightmap = obj.getHeightmap(map_id);
      sz = size(heightmap.X);
      [~, m0] = min(abs(heightmap.Y(:,1) - xy(2)));
      [~, n0] = min(abs(heightmap.X(1,:) - xy(1)));
      i0 = sub2ind(sz, m0, n0);
    end

    function obj = addHeightmap(obj, id, heightmap)
      if isa(heightmap, 'RigidBodyHeightMapTerrain')
        [X, Y] = meshgrid(heightmap.x, heightmap.y);
        [Z, normals] = heightmap.getHeight([reshape(X, 1, []); reshape(Y, 1, [])]);
        heightmap = iris.terrain_grid.Heightmap(X, Y, reshape(Z, size(X)), normals);
      elseif isa(heightmap, 'iris.terrain_grid.Heightmap')
      else
        error('IRIS:TerrainSegmentation:BadHeightmap', 'unrecognized heightmap class: %s', class(heightmap));
      end
      obj.heightmaps(id) = heightmap;
      obj.map_id_last_added(end+1) = id;
      obj.cleanup();
    end

    function b = hasHeightmap(obj, id)
       b = obj.heightmaps.isKey(id);
     end

    function heightmap = getHeightmap(obj, id)
      heightmap = obj.heightmaps(id);
      obj.map_id_last_used(obj.map_id_last_used == id) = [];
      obj.map_id_last_used(end+1) = id;
      obj.cleanup();
    end

    function cleanup(obj)
      if length(obj.map_id_last_added) >= 2 * obj.max_stored_maps || length(obj.map_id_last_used) >= 2 * obj.max_stored_maps
        last_used = obj.map_id_last_used(max(1, end-(obj.max_stored_maps-1)):end);
        last_added = obj.map_id_last_added(max(1, end-(obj.max_stored_maps-1)):end);
        ids = num2cell(union(last_added, last_used));
        obj.heightmaps = containers.Map(ids, obj.heightmaps.values(ids));
      end
    end
  end
end





    