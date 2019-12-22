classdef Ellipsoid
  properties
    C
    d
  end

  methods 
    function obj = Ellipsoid(C, d)
      obj.C = C;
      obj.d = d;
    end
  end
end
