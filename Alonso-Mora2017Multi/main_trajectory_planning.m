function trajectory_planning()
%% 
close all
clear
%% requirements
% given obstacle field O, start configuration z_s with centroid s and destination g for the formation’s centroid.
import iris.inflate_region.*
import iris.drawing.*
import iris.thirdParty.polytopes.*
% constants
lcx = 0.6; lcy = 0.6; lcz = 0.2; % size of object
lmx = 0.6; lmy = 0.4; lmz = 0.3; lm_b = 0.15; % size of platform
% obstacles
xo1 = [3.75,4.75,0.5,0,0,0];
xo2 = [8.25,3,0.5,0,0,0];
obst(1) = pkgMechanics.RigidCuboid(1,xo1,[1.5,3.5,1]);
obst(2) = pkgMechanics.RigidCuboid(1,xo2,[1.5,1,1]);
% obstacles cell
obstcell = createobstcell(obst);
% range
range.lb = [0;0];
range.ub = [10;10];
%% initialize an empty graph
nodez = []; % z vector in row
nodepoly = []; % polytope struct
nodezinpoly = {}; % z index
% initialize edges
edgez = []; % z index
%% configuration
a0 = 0.7; w0 = 0.5;
config = [a0,w0];
%% generate start and goal
start = [1.5,1.5];     % start position
goal = [8.5,8.5];     % goal position 
[nodez, nodepoly, nodezinpoly]= startxgoal(nodez,nodepoly,nodezinpoly,[start;goal],config,obstcell,range,[lcx,lcy,lcz],[lmx,lmy,lmz]);
%% detemine z
z0 = [start,config];
[xr0,xc0,xm0]= z2x(z0,[lcx,lcy,lcz],[lmx,lmy,lmz]);
%% build object 
cub = pkgMechanics.RigidCuboid(1,xc0,[lcx,lcy,lcz]);
%% build platforms
for i=1:4
    plat(i) = pkgMechanics.RigidCuboid(1,xm0(i,:),[lmx,lmy,lmz]);
end
%% build robot arms
Tc2e = lc2Tce([lcx,lcy,lcz]); % => constants
Tm2b = transl([lm_b,0,lmz/2]); % => constants
mdl_puma560
for i=1:4
    rob(i) = SerialLink('name','robot');
    copy(rob(i),p560); rob(i).name = ['robot',num2str(i)];
    rob(i).base = x2T(xm0(i,:))*Tm2b;
    q0(i,:) = rob(i).ikine6s(x2T(xc0)*Tc2e{i},'ru');% right hand elbow up
end
%% visualize map
figure; 
draw_region_2d(nodepoly,obstcell.vert,range);
drawplat3d(nodez,[lcx,lcy,lcz],[lmx,lmy,lmz])
%% add goal node to graph
[nodez,nodezinpoly,nodegrid,edgez]=creategraph(nodez,nodepoly,nodezinpoly,edgez,1,config,range,[lcx,lcy,lcz],[lmx,lmy,lmz]);
[noderoute,polyroute] = shortestpath(nodez,nodezinpoly,nodegrid,1,2);
%% generate random nodes
counter = 0;
while isempty(noderoute)
    nodenum = 1;
    [nodez,nodepoly,nodezinpoly] = randnode(nodez,nodepoly,nodezinpoly,nodenum,config,obstcell,range,[lcx,lcy,lcz],[lmx,lmy,lmz]);
    % create undirected graph and its edges
    [nodez,nodezinpoly,nodegrid,edgez]=creategraph(nodez,nodepoly,nodezinpoly,edgez,nodenum,config,range,[lcx,lcy,lcz],[lmx,lmy,lmz]);
    [noderoute,polyroute] = shortestpath(nodez,nodezinpoly,nodegrid,1,2);
    counter = counter + 1;
end
draw_region_2d(nodepoly(polyroute),[],range);
drawplat3d(nodez(noderoute,:),[lcx,lcy,lcz],[lmx,lmy,lmz]);
hold on
x1=nodez(noderoute,1);
y1=nodez(noderoute,2);
plot(x1,y1,'r-');
hold off
if ~exist('shortestpath.mat', 'file')
    save('shortestpath.mat','nodez','nodepoly','edgez','noderoute','polyroute');
end
%%%%%%%%% the above is the whole global planning part %%%%%%%%%%%


%%%%%%%%% the following is the animation %%%%%%%%%%
load('shortestpath2.mat','nodez','nodepoly','edgez','noderoute','polyroute');
%% save data
% trajectory
dt = 0.5; tacc = 1;
[zarray, dzarray, tarray] = calctraj(nodez(noderoute,:),dt,tacc);
%% visualize map
figure; 
xlim([range.lb(1) range.ub(1)]);
ylim([range.lb(2) range.ub(2)]);
hold on
% % obstacles
% draw_region_2d([],obstcell.vert,range);
% fig.obst = patch('Vertices',obst.verticesStates.position','Faces',obst.faces,'FaceColor','g','FaceAlpha',0.5);
% pause(0.2)
% % object
% fig.cub = patch('Vertices',cub.verticesStates.position','Faces',cub.faces,'FaceColor','m','FaceAlpha',0.5);
% % robot arm and base
zlim([0 8]); plotaxis = gca;
plotopt = {'noname', 'noraise', 'nobase', 'noshadow', 'notiles', 'nowrist', 'nojaxes', 'delay', 0, 'workspace',[plotaxis.XLim,plotaxis.YLim,plotaxis.ZLim]};
% for i=1:4
%     fig.plat(i) = patch('Vertices',plat(i).vertw,'Faces',plat(i).face,'FaceColor','y','FaceAlpha',0.5);
%     rob(i).plot(q0(i,:),plotopt{:});
% end
% pause(0.2)
% % polytope
% for i=1:length(polyroute)
%     draw_region_2d(nodepoly(polyroute(i)),[],range);
%     pause(0.2)
% end
% % graph
% for kk=1:2:size(edgez,1)
%     x1=[nodez(edgez(kk,1),1);nodez(edgez(kk,2),1)];
%     y1=[nodez(edgez(kk,1),2);nodez(edgez(kk,2),2)];
%     line(x1,y1);
% end
% pause(0.2)
% % path
% plot(nodez(noderoute(end),1),nodez(noderoute(end),2),'kd', 'LineWidth', 2)
% plot(nodez(noderoute,1),nodez(noderoute,2),'k-', 'LineWidth', 2)
% pause(0.2)
% hold off
% % obstacles
% cla
% hold on
draw_region_2d([],obstcell.vert,range); 
for i=1:length(obst)
    fig.obst = patch('Vertices',obst(i).verticesStates.position','Faces',obst(i).faces,'FaceColor','g','FaceAlpha',0.5);
end
% object
fig.cub = patch('Vertices',cub.verticesStates.position','Faces',cub.faces,'FaceColor','m','FaceAlpha',0.5);
% robot arm and base
for i=1:4
    fig.plat(i) = patch('Vertices',plat(i).verticesStates.position','Faces',plat(i).faces,'FaceColor','b','FaceAlpha',0.5);
    rob(i).plot(q0(i,:),plotopt{:});
end
% path
plot(nodez(noderoute(end),1),nodez(noderoute(end),2),'rd', 'LineWidth', 2)
plot(nodez(noderoute,1),nodez(noderoute,2),'r-', 'LineWidth', 2)
pause(0.2)
view(3)
hold off
%% animation loop
tic;
playspeed = 1;
while toc<tarray(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    [z,dz] = getviatnow(zarray, dzarray, tarray, tnow);
    [xr,xc,xm] = z2x(z,[lcx,lcy,lcz],[lmx,lmy,lmz]);
    % update object
    cub.updateStates(xc',zeros(6,1));
    % set robot arm state 
    for i=1:4
        rob(i).base = x2T(xm(i,:))*Tm2b;
        q(i,:) = rob(i).ikine6s(x2T(xc)*Tc2e{i});
        plat(i).updateStates(xm(i,:)',zeros(6,1));
    end
    % animation
    set(fig.cub,'Vertices',cub.verticesStates.position','Faces',cub.faces);
    for i=1:4
        set(fig.plat(i),'Vertices',plat(i).verticesStates.position','Faces',plat(i).faces);
        rob(i).animate(q(i,:));
    end
    drawnow
end
end

function obstacle = createobstcell(obstcub)
n_obs=length(obstcub);
for i=1:n_obs
    obstacle_pts(:,:,i) = obstcub(i).verticesStates.position(1:2,1:4);
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
end

function [nodez, nodepoly, nodezinpoly]= startxgoal(nodez,nodepoly,nodezinpoly,randnode,nodeconfig,obstacle,range,lc,lm)
% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% generate nodes outside obstacles
randnum = size(randnode,1);
for i=1:randnum
    % generate ploytope
    randpoly = polytope(obstacle.calc,randnode(i,:)',range);
    % check whether formation exists in a polytope
    [randz,fg] = formationP2z(randpoly,[randnode(i,:),nodeconfig],range,lc,lm);
    if fg==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodezinpoly{polynum+i} = znum+i;
        %%%%%
%         hold on
%         plot(randlocation(1),randlocation(2),'go');
%         plot(randz(1),randz(2),'r*');
%         draw_region_2d(randpoly,[],range);
%         drawplat3d(randz);
%         hold off
        %%%%%
    end
end
end

function [nodez, nodepoly, nodezinpoly]= randnode(nodez,nodepoly,nodezinpoly,nodenum,nodeconfig,obstacle,range,lc,lm)
% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% generate nodes outside obstacles
ctr=1; % counter
while (ctr<=nodenum)
    % generate random two number in range of map's border
    randlocation = [rand* (range.ub(1)-range.lb(1)) + range.lb(1);
        rand* (range.ub(2)-range.lb(2)) + range.lb(2)];
    % if this node is not inside any obstacle
    if ~inpolygon(randlocation(1),randlocation(2),obstacle.poly(1,:),obstacle.poly(2,:))
        % chech if inside a existing polytope
        if ~innodepoly(randlocation,nodepoly)
            % generate ploytope
            randpoly = polytope(obstacle.calc,randlocation,range);
            % check whether formation exists in a polytope
            [randz,fg] = formationP2z(randpoly,[randlocation',nodeconfig],range,lc,lm);
            if fg==1
                % add this location to nodelocation list
                nodez = [nodez;randz];
                nodepoly = [nodepoly,randpoly];
                nodezinpoly{polynum+ctr} = znum+ctr;
                %%%%%
%                 hold on
%                 plot(randlocation(1),randlocation(2),'go');
%                 plot(randz(1),randz(2),'r*');
%                 draw_region_2d(randpoly,[],range);
%                 drawplat3d(randz);
%                 hold off
                %%%%%
                ctr=ctr+1;
            end
        end
    end
end
end

function flag = innodepoly(randlocation,nodepoly)
polynum = length(nodepoly);
count = 0;
for i=1:polynum
    if nodepoly(i).A*randlocation-nodepoly(i).b<=0
        count = count+1;
    end
end
if count==0
    flag = false;
else
    flag = true;
end
end
