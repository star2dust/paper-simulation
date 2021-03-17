close all
clear

% parameters
num = 5;
dim = 2;
L =  [4    -1    -1    -1    -1;
    -1     3    -1    -1     0;
    -1    -1     3     0    -1;
    -1    -1     0     3    -1;
    -1     0    -1    -1     3];
P = diag([kron(ones(1,num-1),[1,zeros(1,num)]),1]);
Pd = kron(P,eye(dim));
Ld = kron(L,eye(num*dim));
w = rand(num,1);
w = w/sum(w);
wd = kron(w,eye(dim));
% states
x = rand(num,dim)'*5;
xs = wd'*x(:);
xh = blkdiag(x(:,1),x(:,2),x(:,3),x(:,4),x(:,5));
xd = [0;0];
u = zeros(size(x));
dxh = zeros(size(xh));
dxd = zeros(size(xd));
uh = dxh;
% plot
gca; axis([-1 1 -0.5 1.5]*5);hold on
fxd = plot(xd(1,:),xd(2,:),'ro');
fxs = plot(xs(1,:),xs(2,:),'bs');
fxh = plot(xh(1,:),xh(2,:),'d',xh(3,:),xh(4,:),'d',xh(5,:),xh(6,:),'d',xh(7,:),xh(8,:),'d',xh(9,:),xh(10,:),'d');
fx = plot(x(1,:),x(2,:),'bo');hold off
% simu
dt = 0.01;
T = 30;
loop = 0;
for t=0:dt:T
    loop = loop+1;
    dxd = ones(size(dxd)).*[cos(pi*t/2);sin(pi*t/2)]*3;
    % estimation
    kc = 1; ko = kc*num/3+1;
    dxh(:) = -ko*Ld*xh(:)+ko*Pd*(kron(ones(num,1),x(:))-xh(:))+uh(:);
    % control
    for i=1:num
        for j=1:num
            uh((i-1)*dim+(1:2),j) = w(i)/(w'*w)*(dxd-kc*(wd'*xh(:,j)-xd));
            if i==j
               u(:,j) = uh((i-1)*dim+(1:2),j);
            end
        end
    end
    % save
    tdata(loop) = t;
    xdata(:,:,loop) = x;
    % animate
    set(fxd,'xdata',xd(1,:),'ydata',xd(2,:));
    set(fxs,'xdata',xs(1,:),'ydata',xs(2,:));
    for i=1:num
        set(fxh(i),'xdata',xh(dim*(i-1)+1,:),'ydata',xh(dim*i,:));
    end
    set(fx,'xdata',x(1,:),'ydata',x(2,:));
    drawnow
    % update
    x = x+u*dt;
    xh = xh+dxh*dt;
    xd = xd+dxd*dt;
    xs = wd'*x(:);
end
