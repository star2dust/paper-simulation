function T = x2T(x)
% T: Homogeneous Transformation Matrix
% x: [xyz rpy] in a row vector
T = transl(x(1:3))*rpy2tr(x(4:end)); % rpy input should be row vector
end