close all
clear

import iristools.*
import rvctools.*

%% get robot and obstacle parameters
if ~exist('robotxobstacle.mat', 'file')
    % If NAME specifies a file with a non-registered file extension 
    % (.mat, .fig, .txt), include the extension.
    creatrobotxobstacle;
end
load('robotxobstacle.mat','range','obstacle');
% visualize map
figure; 
drawregion2d([],obstacle.vert,range);
% desired configure 
as = 0.7; ws = 0.5; 
nodeconfig = [as,ws];
%% initialize an empty graph
nodez = []; % z vector in row
nodepoly = []; % polytope struct
nodezinpoly = {}; % z index
% initialize edges
edgez = []; % z index
%% generate start and goal
if ~exist('startxgoal.mat', 'file')
    nodenum = 2;
    [nodez,nodepoly,nodezinpoly] = randnode(nodez,nodepoly,nodezinpoly,nodenum,nodeconfig,obstacle,range);
    save('startxgoal.mat','nodez','nodepoly','nodezinpoly');
end
load('startxgoal.mat','nodez','nodepoly','nodezinpoly');
drawregion2d(nodepoly,[],range);
drawplat3d(nodez);
%% add new nodes to graph
[nodez,nodezinpoly,nodegrid,edgez]=creategraph(nodez,nodepoly,nodezinpoly,edgez,1,nodeconfig,range);
[noderoute,polyroute] = shortestpath(nodez,nodezinpoly,nodegrid,1,2);
%% generate random nodes
counter = 0;
while isempty(noderoute)
    nodenum = 1;
    [nodez,nodepoly,nodezinpoly] = randnode(nodez,nodepoly,nodezinpoly,nodenum,nodeconfig,obstacle,range);
    % create undirected graph and its edges
    [nodez,nodezinpoly,nodegrid,edgez]=creategraph(nodez,nodepoly,nodezinpoly,edgez,nodenum,nodeconfig,range);
    [noderoute,polyroute] = shortestpath(nodez,nodezinpoly,nodegrid,1,2);
    counter = counter + 1;
end
drawregion2d(nodepoly(polyroute),[],range);
drawplat3d(nodez(noderoute,:));
hold on
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'r-');
hold off
if ~exist('startxgoal.mat', 'file')
    nodenum = 2;
    [nodez,nodepoly,nodezinpoly] = randnode(nodez,nodepoly,nodezinpoly,nodenum,nodeconfig,obstacle,range);
    save('startxgoal.mat','nodez','nodepoly','nodezinpoly');
end
save('shortestpath.mat','nodez','nodepoly','edgez','noderoute','polyroute');





