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
    -1,-1];
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
axis([-2 2 -2 2]);

%% calculate stress matrix
E = [];
for i=1:n 
    E = [E;P'*H'*diag(H(:,i))];
end
[U,S,V] = svd(P);
% the order in S: from big to small
U1 = U(:,1:d+1);
U2 = U(:,d+2:end);
% in fact z is exactly the same as omega given in Fig. 3
z = null(E);
% check if all(svd(U2'*H'*diag(z)*H*U2)>0), then
Omega = H'*diag(z)*H;

%% initial position
x = rand(n,d)*4-2; 
figure
for i=1:m
    fig_edge(i) = plot(x(edge(:,i),1),x(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    fig_node(i) = plot(x(i,1),x(i,2),'.','markersize',50);
end
axis([-2 2 -2 2]);

%% simulation
dt = 0.01;
T = 10;
for t=0:dt:T
    % followers apply control law (11)
    alpha = 50;
    dx = -alpha*Omega*x;
    % first 3 agents are leaders
    dx(1:3,:) = -(x(1:3,:)-r(1:3,:));
    % update states
    x = x+dt*dx;
    % update figure
    for i=1:m
        set(fig_edge(i),'xdata',x(edge(:,i),1),'ydata',x(edge(:,i),2));
    end
    for i=1:n
        set(fig_node(i),'xdata',x(i,1),'ydata',x(i,2));
    end
    drawnow
end