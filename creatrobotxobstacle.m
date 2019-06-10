% generate robot and obstacle parameters
close all
clear

import rvctools.*

%% robot
% constants
lcx = 0.6; lcy = 0.6; lcz = 0.2; max_aw = 1.0; min_aw = 0.45; % size of object
lmx = 0.8; lmy = 0.4; lmz = 0.3; lm_b = 0.15; % size of platform
% get transformation Tc2e
[Tc2e,Tr2d] = lc2Tce([lcx,lcy,lcz]); % => constants
% plat template
platcuboid = RigidCuboid(1,zeros(6,1),[lmx,lmy,lmz]);
platvert = platcuboid.vertices;
% platform frame to base frame
Tm2b = transl([lm_b,0,lmz/2]); % => constants
%% range
lb = [-4;-4]; ub = [8;8];
range.lb = lb;
range.ub = ub;
%% obstacle
% options of obstacles
n_obs = 2;
randobs = false;
% obstacles vertices
if ~randobs
    obstacle_pts(:,:,1) =   [-1.3023    1.3371   -1.6865   -4.5655;
        5.7248    4.7600    9.0761    9.1427];
    obstacle_pts(:,:,2) =   [1.0132    3.8220    2.2281    5.4110;
        1.0316    1.0881    0.7004   -3.5495];
else
    % Generate a randomly distributed field of random obstacles. Each
    % obstacle consists of 2^dim points uniformly randomly distributed
    % within a 2*scale-length cube around a randomly selected center point
    scale = 1; dim = 2; 
    offsets = scale * (rand(dim, 2^dim * n_obs) * 2 - 1);
    centers = bsxfun(@plus, bsxfun(@times, rand(dim, n_obs), ub - lb), lb);
    centers = reshape(centers, dim, 1, []);
    obstacle_pts = bsxfun(@plus, centers, reshape(offsets ./ (n_obs^(1/dim)), [dim, 2^dim, n_obs]));
end
% reset obstacles vertices in its convexhull (iris requires obstacles to be convex)
obsidx = cell(1,size(obstacle_pts,3));
obstacle.vert = cell(1,size(obstacle_pts,3));
for j = 1:n_obs
    obs = obstacle_pts(:,:,j);
    if size(obs, 2) > 1
        if size(obs, 2) > 2
            obsidx{j} = convhull(obs(1,:), obs(2,:));
        else
            obsidx{j} = [1,2,1];
        end
    end
    obstacle.vert{j} = obs(:,obsidx{j})';
    obstacle.calc{j} = obs(:,obsidx{j});
end
% obstacles cell
obstacle.poly = obstacle.vert{1}';
for i=2:size(obstacle.vert,1)
    obstacle.poly = [obstacle.poly,NaN(2,1),obstacle.vert{i}'];
end

save('robotxobstacle.mat','obstacle','range','lcx','lcy','lcz','lmx','lmy','lmz','lm_b','max_aw','min_aw','Tc2e','Tr2d','Tm2b','platvert');