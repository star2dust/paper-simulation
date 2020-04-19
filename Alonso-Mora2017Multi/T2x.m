function x = T2x(T)
% T: Homogeneous Transformation Matrix
% x: [xyz rpy] in a row vector
[~,t] = tr2rt(T);
x = [t',tr2rpy(T)];
end