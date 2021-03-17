close all
clear

%% topology graph
D = [1,-1,0,0,0,0,0,0,0,-1,0,1;
    -1,0,0,0,0,0,1,-1,0,0,0,0;
    0,1,-1,0,0,0,0,0,1,0,0,0;
    0,0,0,0,0,1,-1,0,0,1,-1,0;
    0,0,1,-1,0,0,0,0,0,0,1,-1;
    0,0,0,0,1,-1,0,0,-1,0,0,0;
    0,0,0,1,-1,0,0,1,0,0,0,0];
L = D*D';
H = D';

%% dimensions
[n,m] = size(D);
d = 2;
% r represents P(r)
r = [2,0;
    1,1;
    1,-1;
    0,1;
    0,-1;
    -1,1;
    -1,-1]+2;
% P represents \bar P(r)
P = [r,ones(n,1)];
% edge set
edge = mod(reshape(find(D~=0),2,m),n);
edge(edge==0) = n;

%% target formation
figure
for i=1:m
    plot(r(edge(:,i),1),r(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    plot(r(i,1),r(i,2),'.','markersize',50);
end
axis equal;

%% affine transformation
A = rand(d,d);
b = rand(d,1);
ra = r';
ra(:) = kron(eye(n),A)*vec(r')+kron(ones(n,1),b);
ra = ra';
figure
for i=1:m
    plot(ra(edge(:,i),1),ra(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    plot(ra(i,1),ra(i,2),'.','markersize',50);
end
axis equal;
Pi = eye(n)-P*pinv(P);

%% calculate stress matrix
E = [];
for i=1:n 
    E = [E;P'*H'*diag(H(:,i))];
end
[U,S,V] = svd(P);
% the order in S: from big to small
U1 = U(:,1:d+1);
U2 = U(:,d+2:end);
% in fact w is exactly the same as omega given in Fig. 3
w = null(E);
% check if all(svd(U2'*H'*diag(w)*H*U2)>0), then
Omega = H'*diag(w)*H;

%% initial position
x = (rand(n,d)-0.5)*4; 
figure
for i=1:m
    fig_edge(i) = plot(x(edge(:,i),1),x(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    fig_node(i) = plot(x(i,1),x(i,2),'.','markersize',50);
end
axis([-2 2 -2 2]);

%% auxiliary variable
aux = false;
y = rand(n,d);
z = rand(n,d);
DP = [];
for i=1:size(P,1)
    DP = blkdiag(DP,P(i,:));
end
Dr_inv = diag(vec(P'))^-1;
v = pinv(DP)*x;
dv = v*0;

%% simulation
dt = 0.001;
T = 5;
loop = 0;
playspeed = 5;
for t=0:dt:T
    loop = loop+1;
    % followers apply control law
    alpha = 1; beta = alpha*2;
    % consensus algorithms
%     dx = -alpha*L*sign(L*x);
%     dx = -alpha*sign(L*x);
%     dx = -alpha*D*sign(D'*x);
    % affine formation algorithms
    if ~aux
%         dx = -alpha*Omega*sign(Omega*x);
        K = kron(D,eye(d+1));
        dv = -alpha*Dr_inv*K*sign(K'*v);
        dx = DP*dv;
%         dx = -alpha*DP*Dr_inv*K*sign(K'*pinv(DP)*x);
    else
        dy = -alpha*sign(Omega*y);
        dz = alpha*sign(Omega*y)-beta*D*sign(D'*z);
    end
    % first 3 agents are leaders
%     dx(1:3,:) = -(x(1:3,:)-r(1:3,:));
    % update states
    if ~aux
        x = x+dt*dx;
        v = v+dt*dv;
    else
        y = y+dt*dy;
        z = z+dt*dz;
        x = y+z;
    end
    pos_data(:,:,loop) = x;
    % update figure
    if ~mod(loop,playspeed)
        for i=1:m
            set(fig_edge(i),'xdata',x(edge(:,i),1),'ydata',x(edge(:,i),2));
        end
        for i=1:n
            set(fig_node(i),'xdata',x(i,1),'ydata',x(i,2));
        end  
    end
    drawnow
end
figure 
t_data = 0:dt:T;
xpos_data = squeeze(pos_data(:,1,:));
ypos_data = squeeze(pos_data(:,2,:));
plot3(kron(ones(n,1),t_data)',xpos_data',ypos_data');
xlabel('time/s');ylabel('x/m');zlabel('y/m');grid
figure
plot(kron(ones(n,1),t_data)',xpos_data');
xlabel('time/s');ylabel('x/m');grid
figure
plot(kron(ones(n,1),t_data)',ypos_data');
xlabel('time/s');ylabel('y/m');grid