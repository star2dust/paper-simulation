close all
clear

import rvctools.*
import iristools.*

load('shortestpath.mat','nodez','nodepoly','edgez','noderoute','polyroute');
load('robotxobstacle.mat','range','obstacle','lcx','lcy','lcz','lmx','lmy','lmz','Tm2b','Tc2e');
%% save data
% trajectory
dt = 0.5; tacc = 1;
[zarray, dzarray, tarray] = calctraj(nodez(noderoute,:),dt,tacc);
% z to xc and xm
z0 = zarray(1,:);
[xr0,xc0,xm0] = z2x(z0);
% object
cub = createcuboid(xc0,[lcx,lcy,lcz]);
% platform
for i=1:4
    plat(i) = createcuboid(xm0(i,:),[lmx,lmy,lmz]);
end
% arm
mdl_puma560;
for i=1:4
    rob(i) = SerialLink('name','robot');
    copy(rob(i),p560); 
    rob(i).name = ['robot',num2str(i)];
    rob(i).base = x2T(xm0(i,:))*Tm2b;
    q0(i,:) = rob(i).ikine6s(x2T(xc0)*Tc2e{i},'ru');% right hand elbow up
end
%% visualize map
figure; 
xlim([range.lb(1) range.ub(1)]);
ylim([range.lb(2) range.ub(2)]);
hold on
% obstacles
drawregion2d([],obstacle.vert,range); 
pause(0.2)
% object
fig.cub = patch('Vertices',cub.vertw,'Faces',cub.face,'FaceColor','b','FaceAlpha',0.5);
% robot arm and base
zlim([0 8]); plotaxis = gca;
plotopt = {'noname', 'noraise', 'nobase', 'noshadow', 'notiles', 'nowrist', 'nojaxes', 'delay', 0, 'workspace',[plotaxis.XLim,plotaxis.YLim,plotaxis.ZLim]};
for i=1:4
    fig.plat(i) = patch('Vertices',plat(i).vertw,'Faces',plat(i).face,'FaceColor','y','FaceAlpha',0.5);
    rob(i).plot(q0(i,:),plotopt{:});
end
pause(0.2)
% polytope
for i=1:length(polyroute)
    drawregion2d(nodepoly(polyroute(i)),[],range);
    pause(0.2)
end
% graph
for kk=1:2:size(edgez,1)
    x1=[nodez(edgez(kk,1),1);nodez(edgez(kk,2),1)];
    y1=[nodez(edgez(kk,1),2);nodez(edgez(kk,2),2)];
    line(x1,y1);
end
pause(0.2)
% path
plot(nodez(noderoute(end),1),nodez(noderoute(end),2),'kd', 'LineWidth', 2)
plot(nodez(noderoute,1),nodez(noderoute,2),'k-', 'LineWidth', 2)
pause(0.2)
hold off
% obstacles
cla
hold on
drawregion2d([],obstacle.vert,range); 
% object
fig.cub = patch('Vertices',cub.vertw,'Faces',cub.face,'FaceColor','b','FaceAlpha',0.5);
% robot arm and base
for i=1:4
    fig.plat(i) = patch('Vertices',plat(i).vertw,'Faces',plat(i).face,'FaceColor','y','FaceAlpha',0.5);
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
    [z,dz] = getcurvia(zarray, dzarray, tarray, tnow);
    [xr,xc,xm] = z2x(z);
    % update object
    cub = updatecuboid(cub,xc);
    % set robot arm state 
    for i=1:4
        rob(i).base = x2T(xm(i,:))*Tm2b;
        q(i,:) = rob(i).ikine6s(x2T(xc)*Tc2e{i});
        plat(i) = updatecuboid(plat(i),xm(i,:));
    end
    % animation
    set(fig.cub,'Vertices',cub.vertw,'Faces',cub.face);
    for i=1:4
        set(fig.plat(i),'Vertices',plat(i).vertw,'Faces',plat(i).face);
        rob(i).animate(q(i,:));
    end
    drawnow
end
