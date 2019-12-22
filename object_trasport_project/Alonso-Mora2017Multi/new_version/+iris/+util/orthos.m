function [v1, v2] = orthos(y)

if ~(y(2) == 0 && y(3) == 0)
  w = axis2rotmat([1,0,0,pi/2]) * y;
elseif ~(y(1) == 0 && y(3) == 0)
  w = axis2rotmat([0,1,0,pi/2]) * y;
else
  w = axis2rotmat([0,0,1,pi/2]) * y;
end
v1 = cross(y, w);
v2 = cross(y, v1);

end

