classdef Heightmap
  % A simple container for height map data stored as a grid of heights with associated normal vectors. 
  properties(SetAccess=private, GetAccess=public)
    X;
    Y;
    Z;
    normals;
    resolution;
  end

  methods
    function obj = Heightmap(X, Y, Z, normals)
      if length(X) == numel(X) && length(Y) == numel(Y) 
        [X, Y] = meshgrid(X, Y);
      end
      obj.X = X;
      obj.Y = Y;
      obj.Z = Z;
      obj.normals = normals;
      if ~isempty(X)
        obj.resolution = norm(X(1,2) - X(1,1));
      else
        obj.resolution = nan;
      end
    end

    function slope_angles = getSlopeAngles(obj)
      slope_angles = atan2(sqrt(sum(obj.normals(1:2,:).^2,1)), obj.normals(3,:));
      slope_angles = reshape(slope_angles, size(obj.X));
    end
  end
end


