close all
clear

%% initial terms
% constants
lcx = 0.6; lcy = 0.6; lcz = 0.2; % size of object
lmx = 0.8; lmy = 0.4; lmz = 0.3; lm_b = 0.15; % size of platform
if nargin < 1
    % variables
    w0 = 0.5; a0 = [1,1,1,1]*(lmx/2-lm_b+0.7); th0 = [-pi/6,pi/6,-pi/6,pi/6];
%     w0 = 0.5; a0 = [1,1,1,1]*1; th0 = [0,0,0,0];
    w1 = 0.8; a1 = [1,1,1,1]*(lmx/2-lm_b+0.2); th1 = [0,0,0,0];
    % detemine the center of formation (also the trajectory)
    xr0 = [0,0,0,0,0,pi/4];
    % initial via points
    xrf = [4.5,4.5,0,0,0,0];
    xr1 = [4.5,0,0,0,0,pi/8];
    % intial y variable
    z = [xr0,a0,th0,w0;xr1,a1,th1,w1;xrf,a0,th0,w0];
else
    z = path;
end
% get transformation Tc2e
Tc2e = lc2Tce([lcx,lcy,lcz]); % => constants
%% build object 
% initial formation frame xr0 to object frame xc0 (ground => air)
xc0 = xr2c(xr0,w0);
% build object cuboid
cub = pkgMechanics.RigidCuboid(1,xc0,[lcx,lcy,lcz]);
%% build platforms
% formation frame to platform frame
xm0 = xr2m(xr0,a0,th0,[lcx,lcy,lcz],[lmx,lmy,lmz]);
% platform frame to base frame 
Tm2b = transl([lm_b,0,lmz/2]); % => constants
% build mobile platform cuboid
for i=1:4
    plat(i) = pkgMechanics.RigidCuboid(1,xm0(i,:),[lmx,lmy,lmz]);
end
%% build robot arms
mdl_puma560;
for i=1:4
    rob(i) = SerialLink('name','robot');
    copy(rob(i),p560); 
    rob(i).name = ['robot',num2str(i)];
    rob(i).base = x2T(xm0(i,:))*Tm2b;
    q0(i,:) = rob(i).ikine6s(x2T(xc0)*Tc2e{i},'ru');% right hand elbow up
end
%% generate trajectory
dt = 0.5; tacc = 1;
[yarray, dyarray, tarray] = calctraj(z,dt,tacc);
%% figure
% parameters
if nargin < 2
    plotAxis = [-2 6 -2 6 0 8];
else
    plotAxis = paxis;
end
plotopt = {'noname', 'noraise', 'nobase', 'noshadow', 'notiles', 'nowrist', 'nojaxes', 'delay', 0, 'workspace',plotAxis};
% build figure
figure; 
view(3); axis(plotAxis); grid; hold on
% object
fig.cub = patch('Vertices',cub.verticesStates.position','Faces',cub.faces,'FaceColor','b','FaceAlpha',0.5);
% robot arm and base
for i=1:4
    fig.plat(i) = patch('Vertices',plat(i).verticesStates.position','Faces',plat(i).faces,'FaceColor','y','FaceAlpha',0.5);
    rob(i).plot(q0(i,:),plotopt{:});
end
hold off
%% animation loop
tic;
playspeed = 1;
while toc<tarray(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    [z,dy] = getviatnow(yarray, dyarray, tarray, tnow);
    [xr,a,th,w] = y2xrathw(z);
    [dxr,da,dth,dw] = y2xrathw(dy);
    % update object
    xc = xr2c(xr,w);
    Tc = x2T(xc);
    cub.updateStates(xc',dxr');
    cub.getStates;
    % set robot arm state 
    xm = xr2m(xr,a,th,[lcx,lcy,lcz],[lmx,lmy,lmz]);
    for i=1:4
        rob(i).base = x2T(xm(i,:))*Tm2b;
        q(i,:) = rob(i).ikine6s(Tc*Tc2e{i});
        plat(i).updateStates(xm(i,:)',dxr');
    end
    % animation
    set(fig.cub,'Vertices',cub.verticesStates.position','Faces',cub.faces);
    for i=1:4
        set(fig.plat(i),'Vertices',plat(i).verticesStates.position','Faces',plat(i).faces);
        rob(i).animate(q(i,:));
    end
    drawnow
end
