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

%% trajectory
figure
via = [0,0;
    5,0;
    10,0;
    10,-5;
    10,-10;
    5,-10;
    0,-10];
for j=1:size(via,1)
    if ~mod(j,2)
        if j==4
            T1 = diag([0.1,1]);
        else
            T1 = diag([1,0.1]);
        end
    else
        T1 = eye(2);
    end
    T2 = rot2(-pi/2*floor((j-1)/2));
    ra(:,:,j) = r*T2'*T1'+via(j,:);
    qvia(j,:) = [vec(T1*T2)',via(j,:)];
    for i=1:m
        plot(ra(edge(:,i),1,j),ra(edge(:,i),2,j),'k','linewidth',2); hold on
    end
end
[qr,dqr,ddqr,tr] = mstraj_(qvia,ones(1,6),0.1,0.2);
A = reshape(qr(1,1:4)',[2,2]);
b = qr(1,5:6)';
xr = r*A'+b';
% for j=1:length(tr)
%     dA = reshape(dqr(j,1:4)',[2,2]);
%     db = dqr(j,5:6)';
%     dxr = r*dA'+db';
%     xr = xr+dxr*0.1;
%     plot(xr(1,1),xr(1,2),'bo');
%     plot(xr(2,1),xr(2,2),'go');
%     plot(xr(3,1),xr(3,2),'mo');
%     drawnow
% end


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
Oll = Omega(1:3,1:3);
Olf = Omega(1:3,4:end);
Ofl = Omega(4:end,1:3);
Off = Omega(4:end,4:end);

%% initial position
x = rand(n,d)*4-2; 
dx = zeros(size(x));
% figure
for i=1:m
    fig_edge(i) = plot(x(edge(:,i),1),x(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    fig_node(i) = plot(x(i,1),x(i,2),'.','markersize',50);
end
axis([-2 12 -12 2]);

%% simulation
dt = 0.01;
loop = 0;
for t=tr(1):dt:tr(end)
    loop = loop+1;
    dq = interp1(tr,dqr,t);
    dA = reshape(dq(1:4)',[2,2]);
    db = dq(5:6)';
    dxr = r*dA'+db';
    % test leader
    alpha = 1;
%     dx = dxr-alpha*(x-xr);
    % followers apply control law (11)   
    D = H';
    gamma = diag(Omega);
    for i=4:n
        errsum = [0,0];
        edge_ind = find(D(i,:)~=0);
        for k=edge_ind
            node_ind = find(D(:,k)~=0);
            j = node_ind(node_ind~=i);
            errsum = errsum+z(k)*(x(i,:)-x(j,:)-dx(j,:));
        end
        dx(i,:) = -1/gamma(i)*errsum;
    end
    % first 3 agents are leaders
    dx(1:3,:) = dxr(1:3,:)-alpha*(x(1:3,:)-xr(1:3,:));
    % update states
    x = x+dt*dx;
    xr = xr+dt*dxr;
    pos_data(:,:,loop) = x;
    % update figure
    for i=1:m
        set(fig_edge(i),'xdata',x(edge(:,i),1),'ydata',x(edge(:,i),2));
    end
    for i=1:n
        set(fig_node(i),'xdata',x(i,1),'ydata',x(i,2));
    end
    drawnow
end
figure 
t_data = tr(1):dt:tr(end);
xpos_data = squeeze(pos_data(:,1,:));
ypos_data = squeeze(pos_data(:,2,:));
plot3(kron(ones(n,1),t_data)',xpos_data',ypos_data');
xlabel('time/s');ylabel('x/m');zlabel('y/m');grid