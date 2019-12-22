classdef CollisionModel
  properties
    foot_shape
    body_slices
  end

  methods
    function obj = CollisionModel(foot_shape, body_slices)
      obj.foot_shape = foot_shape;
      obj.body_slices = body_slices;
    end

  end
end
