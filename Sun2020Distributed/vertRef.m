function [vert_ref,angle_inward,angle_outward] = vertRef(robot_num,vert_affine)
% polytope
angle_central = 2*pi/robot_num;
if ~mod(robot_num,2)
    vert_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)'];
else
    vert_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)'];
end
vert_ref = vert_ref*vert_affine';
angle_inward = cart2pol(-vert_ref(:,1),-vert_ref(:,2));
angle_outward = cart2pol(vert_ref(:,1),vert_ref(:,2));
end