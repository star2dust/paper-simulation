function R = rotmat(theta)
s = sin(theta);
c = cos(theta);
R = [c, -s; s, c];
end

