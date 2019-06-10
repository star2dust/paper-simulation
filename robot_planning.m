function robot_planning(path,paxis)
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
end

function [y,dy] = getviatnow(yarray, dyarray, tarray, tnow)
y = interp1(tarray,yarray,tnow); 
dy = interp1(tarray,dyarray,tnow);
end

function [xr,a,th,w] = y2xrathw(y)
xr = y(1:6); a = y(7:10); th = y(11:14); w = y(15);
end

function [yarray0, dyarray0, tarray] = calctraj(via,dt,tacc)
ptlen = size(via,2);
y0 = via(1,:);
yarray0 = mstraj(via(2:end,:),ones(1,ptlen),[],y0,dt,tacc); % row vector for each xr
yarray0 = [y0;yarray0];
dyarray0 = [diff(yarray0)./dt;zeros(1,ptlen)];
tarray = 0:dt:(size(yarray0,1)-1)*dt;
end

function xc0 = xr2c(xr0,w0)
Tr2c0 = transl([0,0,w0]);
xc0 = T2x(x2T(xr0)*Tr2c0);
end

function xm0 = xr2m(xr0,a0,th0,lc,lm)
lcx = lc(1); lcy = lc(2); lmz = lm(3);
n = length(th0);
[~,Tr2d] = lc2Tce(lc);
for i=1:n
    Td2m{i} = trotz(th0(i))*transl([a0(i),0,lmz/2])*trotz(pi);
    xm0(i,:) = T2x(x2T(xr0)*Tr2d{i}*Td2m{i});
end
end

function [Tc2e, Tr2d] = lc2Tce(lc)
lcx = lc(1); lcy = lc(2);
Tr2d{1} = transl([-lcx/2,0,0])*trotz(pi);
Tr2d{2} = transl([0,-lcy/2,0])*trotz(-pi/2);
Tr2d{3} = transl([lcx/2,0,0]);
Tr2d{4} = transl([0,lcy/2,0])*trotz(pi/2);
Tc2e{1} = transl([-lcx/2,0,0])*rpy2tr([pi/2,0,pi/2]);
Tc2e{2} = transl([0,-lcy/2,0])*rpy2tr([pi/2,0,pi]);
Tc2e{3} = transl([lcx/2,0,0])*rpy2tr([pi/2,0,3*pi/2]);
Tc2e{4} = transl([0,lcy/2,0])*rpy2tr([pi/2,0,0]);
end


function T = x2T(x)
% T: Homogeneous Transformation Matrix
% x: [xyz rpy] in a row vector
T = transl(x(1:3))*rpy2tr(x(4:end)); % rpy input should be row vector
end

function x = T2x(T)
% T: Homogeneous Transformation Matrix
% x: [xyz rpy] in a row vector
[~,t] = tr2rt(T);
x = [t',tr2rpy(T)];
end

function xe = xc2xe(xc,xc_e)
trl_c_e = xc_e(1:3);
rpy_c_e = xc_e(4:end);
Tc = x2T(xc);
Rc_e = rpy2r(rpy_c_e);
Tc2e = rt2tr(Rc_e,trl_c_e);
xe = T2x(Tc*Tc2e);
end

function dxe = dxc2dxe(dxc,xc,xc_e)
trl_c_e = xc_e(1:3)';
rpy_c_e = xc_e(4:end)';
% get velocity
wx = [1,0,0]; wy = [0,1,0]; wz = [0,0,1];
dtrl_w_c = dxc(1:3)';
drpy_w_c = dxc(4:end)';
dRw_c = dxc(6)*skew(wz)*rotz(xc(6))*roty(xc(5))*rotx(xc(4))+rotz(xc(6))*dxc(5)*skew(wy)*roty(xc(5))*rotx(xc(4))+rotz(xc(6))*roty(xc(5))*dxc(4)*skew(wx)*rotx(xc(4));
dtrl_w_e = dRw_c*trl_c_e+dtrl_w_c;
Rc_e = rotz(rpy_c_e(3))*roty(rpy_c_e(2))*rotx(rpy_c_e(1));% no relative rotation between c and e, dRc_e=zeros(3);
drpy_w_e = Rc_e'*drpy_w_c;
dxe = [dtrl_w_e;drpy_w_e];
end

function ddxe = ddxc2ddxe(ddxc,dxc,xc,xc_e)

end

function dq = dxe2dq(rob,dxe,q)
Jac = rob.jacob0(q);
dq = Jac^-1*dxe;
end
