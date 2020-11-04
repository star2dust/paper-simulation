close all
clear

load('shortestpath2.mat','nodez','nodepoly','edgez','noderoute','polyroute');
%% save data
% constants
lcx = 0.6; lcy = 0.6; lcz = 0.2; % size of object
lmx = 0.8; lmy = 0.4; lmz = 0.3; lm_b = 0.15; % size of platform
% obstacles
xo1 = [3.75,4.75,0.5,0,0,0];
xo2 = [8.25,3,0.5,0,0,0];
obst(1) = pkgMechanics.RigidCuboid(1,xo1,[1.5,3.5,1]);
obst(2) = pkgMechanics.RigidCuboid(1,xo2,[1.5,1,1]);
% range
range.lb = [0;0];
range.ub = [10;10];
% trajectory
dt = 1.5; tacc = 2;
[zarray, dzarray, tarray] = calctraj(nodez(noderoute,:),dt,tacc);
z0 = zarray(1,:);
dz0 = dzarray(1,:);
[xr0,xc0,xm0]= z2x(z0,[lcx,lcy,lcz],[lmx,lmy,lmz]);
%% build object 
mc = 0.05;
cub = pkgMechanics.RigidCuboid(mc,xc0,[lcx,lcy,lcz]);
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
    xe(i,:) = T2x(x2T(xc0)*Tc2e{i});
    q0(i,:) = rob(i).ikine6s(x2T(xc0)*Tc2e{i},'ru');% right hand elbow up
end
%% visualize map
figure; 
xlim([range.lb(1) range.ub(1)]);
ylim([range.lb(2) range.ub(2)]);
hold on
zlim([0 8]); plotaxis = gca;
plotopt = {'noname', 'noraise', 'nobase', 'noshadow', 'notiles', 'nowrist', 'nojaxes', 'delay', 0, 'workspace',[plotaxis.XLim,plotaxis.YLim,plotaxis.ZLim]};
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
% endeffector
fig.ende = plot3(xe(:,1),xe(:,2),xe(:,3),'ko');
% path
plot(nodez(noderoute(end),1),nodez(noderoute(end),2),'rd', 'LineWidth', 2)
plot(nodez(noderoute,1),nodez(noderoute,2),'r-', 'LineWidth', 2)
view(3)
hold off
%% controller and ode45
fd1 = 0.05*[1, 0]';
fd2 = 0.05*[0, 1]';
fd3 = 0.05*[-1, 0]';
fd4 = 0.05*[0, -1]';
fd = [fd1,fd2,fd3,fd4];
for i=1:4
    xe0(i,:) = T2x(x2T(xc0)*Tc2e{i});
    xxi0(:,i) = xe0(i,1:2)';
    mi(i) = 1;
end
dxxi0 = kron(ones(1,4),dz0(1:2)');
xxc0 = xc0(1:2)';
dxxc0 = dz0(1:2)';
% centralized control
% y0 = [xxi0(:);dxxi0(:);xxc0(:);dxxc0(:)];
% [tyarray,yarray] = ode45(@(t,y) ydot(t,y,zarray,dzarray,tarray,fd,mi,mc,[lcx,lcy,lcz]),[0 tarray(end)],y0);
% distributed control
vi = kron(ones(1,4),dz0(1:2)'); % robot 1 is leader
y0 = [xxi0(:);dxxi0(:);xxc0(:);dxxc0(:);vi(:)];
[tyarray,yarray] = ode45(@(t,y) ydot2(t,y,zarray,dzarray,tarray,fd,mi,mc,[lcx,lcy,lcz]),[0 tarray(end)],y0);
%% animation loop
% vi=VideoWriter('robot_manipulation');
% open(vi);
tic;
playspeed = 1;
while toc<tarray(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    z = interp1(tarray,zarray,tnow); 
    dz = interp1(tarray,dzarray,tnow);
    y = interp1(tyarray,yarray,tnow); 
    [xr,xc,xm] = z2x(z,[lcx,lcy,lcz],[lmx,lmy,lmz]);
    % update object
    xc(1:2) = y(17:18);
    xe = reshape(y(1:8),2,4)';
    xe(:,3) = xc(3)*ones(4,1);
    cub.updateStates(xc,zeros(6,1));
    % set robot arm state 
    for i=1:4
        rob(i).base = x2T(xm(i,:))*Tm2b;
        q(i,:) = rob(i).ikine6s(xe(i,:));
        plat(i).updateStates(xm(i,:)',zeros(6,1));
    end
    % animation
    set(fig.cub,'Vertices',cub.verticesStates.position','Faces',cub.faces);
    set(fig.ende,'XData',xe(:,1),'YData',xe(:,2),'ZData',xe(:,3));
    for i=1:4
        set(fig.plat(i),'Vertices',plat(i).verticesStates.position','Faces',plat(i).faces);
        rob(i).animate(q(i,:));
    end
%     frame=getframe(gcf); % or use drawnow
%     writeVideo(vi,frame);
    drawnow
end
% close(vi);