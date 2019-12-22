function drawPolyFromVertices(c, color, varargin)

if isempty(c)
  return
end

k = convhull(c(1,:), c(2,:), c(3,:));
X = reshape(c(1,k'), size(k'));
Y = reshape(c(2,k'), size(k'));
Z = reshape(c(3,k'), size(k'));
plot3(X,Y,Z,color)
fill3(X,Y,Z,color, 'FaceAlpha', 0.5, varargin{:})