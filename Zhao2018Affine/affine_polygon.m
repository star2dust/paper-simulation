close all
clear

% number
n = 5;
% topology graph
D = [-1,-1,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1;
      1, 0, 0, 0,-1,-1, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0;
      0, 0, 0, 1, 0, 0, 1, 1, 0, 0,-1, 0,-1, 0, 0, 0, 0;
      0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,-1,-1, 0, 0;
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0,-1, 0;
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1];  
n_rm = size(D,1)-n;
if n_rm>0
  for i=1:n_rm
      D(:,D(end,:)~=0) = [];
      D(end,:) = [];
  end
end
L = D*D';
H = D';
[n,m] = size(D);
% r represents P(r)
ang = 2*pi/n;
if ~mod(n,2)
    r = [cos([0 kron(1:(n-1)/2,[1,-1]) n/2]*ang)',...
        sin([0 kron(1:(n-1)/2,[1,-1]) n/2]*ang)'];
else
    r = [cos([0 kron(1:(n-1)/2,[1,-1])]*ang)',...
        sin([0 kron(1:(n-1)/2,[1,-1])]*ang)'];
end
d = size(r,2);
% P represents \bar P(r)
P = [r,ones(n,1)];
% edge set
edge = mod(reshape(find(D~=0),2,m),n);
edge(edge==0) = n;
% target formation
figure
for i=1:m
    plot(r(edge(:,i),1),r(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    plot(r(i,1),r(i,2),'.','markersize',50);
    text(r(i,1),r(i,2),num2str(i),'horizontalalignment','center','color','w');
end
axis([-2 2 -2 2]);
% calculate stress matrix
E = [];
for i=1:n 
    E = [E;P'*H'*diag(H(:,i))];
end
[U,S,V] = svd(P);
% the order in S: from big to small
U1 = U(:,1:d+1);
U2 = U(:,d+2:end);
% calculate M
z = null(E);
c = zeros(size(z,2),1);
for i=1:size(z,2)
    N{i} = U2'*H'*diag(z(:,i))*H*U2;
end
% LMI solver
setlmis([]);
for i=1:size(z,2)
    c(i) = lmivar(1,[1,0]);
    lmiterm([-1,1,1,c(i)],1,N{i});
end
lmi = getlmis;
[tmin,csol] = feasp(lmi);
for i=1:size(z,2)
   c(i) = dec2mat(lmi,csol,c(i)); 
end
M = [N{:}]*kron(c,eye(size(N{1},2)));
% check if all(svd(U2'*H'*diag(z)*H*U2)>0), then
M(M<10^-10) = 0;
if all(eig(M)>0)
    w = z*c;
    Omega = H'*diag(w)*H;
    disp('M is positive definite.')
else
    error('M is not positive definite.')
end

%% initial position
x = rand(n,d)*4-2; 
vl = 1:n; vl = vl(1:d+1);
x(vl,:) = r(vl,:);
figure
for i=1:m
    fig_edge(i) = plot(x(edge(:,i),1),x(edge(:,i),2),'k','linewidth',2); hold on
end
for i=1:n
    fig_node(i) = plot(x(i,1),x(i,2),'.','markersize',50);
    fig_text(i) = text(x(i,1),x(i,2),num2str(i),'horizontalalignment','center','color','w');
end
axis([-2 2 -2 2])
% simulation
dt = 0.01;
T = 20;
for t=0:dt:T
    % followers apply control law (11)
    alpha = 1;
    dx = -alpha*Omega*x;
    % first 3 agents are leaders
    dx(vl,:) = 0;
    % update states
    x = x+dt*dx;
    % update figure
    for i=1:m
        set(fig_edge(i),'xdata',x(edge(:,i),1),'ydata',x(edge(:,i),2));
    end
    for i=1:n
        set(fig_node(i),'xdata',x(i,1),'ydata',x(i,2));
        set(fig_text(i),'position',[x(i,1),x(i,2)]);
    end
    drawnow
end