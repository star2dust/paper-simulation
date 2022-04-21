close all
clear

% x0 = [0.3658    0.9727;
%     0.7635    0.1920;
%     0.6279    0.1389;
%     0.7720    0.6963;
%     0.9329    0.0938];
% x0 = rand(5,2);
num = 5;
x0 = vertRef(num,eye(2));
gph = bigraph(num,x0);
L = gph.laplacian;
S = gph.stress;
D = gph.incidence;
A = gph.adjacency;
K = A./6;
K = diag(1-sum(K,2))+K;
xd = vertRef(num,rand(2,2))+rand(1,2);
% xd = [0.1847    0.4460;
%     0.8786    0.8901;
%    -0.5912   -0.4346;
%     0.5315    0.2840;
%    -0.3769   -0.5347];
w = rand(num,1);
w = w/sum(w);
x_opt = sum(w)^-1*sum(diag(w)*xd);
dxd = zeros(size(xd));
% plot
x = rand(num,2);
dx = zeros(size(x));
y = x-xd;
dy = zeros(size(y));
z = zeros(size(y));
dz = zeros(size(y));
v = zeros(size(y));
dv = zeros(size(y));
gca; axis([-1 1 -1 1]*5);hold on
fx0 = plot(x0(:,1),x0(:,2),'co');
fxd = plot(xd(:,1),xd(:,2),'ro');
fxo = plot(x_opt(:,1),x_opt(:,2),'rx');
fx = plot(x(:,1),x(:,2),'bo');
fy = plot(y(:,1),y(:,2),'go');hold off
% simu
dt = 0.01;
T = 10;
loop = 0;
algo = 6;
for t=0:dt:T
    loop = loop+1;
    switch algo
        case 1
            % DGD (Distributed Gradient Descent)
            a = 3;
            dx = -L*x-a*diag(w)*(x-xd);
        case 2
            % EXTRA(Shi 2015)/Kia 2015 (sum y0 = 0)
            if t==0
                y = zeros(size(x));
            end
            a = 3;
            dx = -L*x-a*diag(w)*(x-xd)+y;
            dy = -L*x;
        case 3
            % primal-dual (equivalent to extra proved by Mokhtari 2016)
            a = 3;
            dx = -L*x-a*diag(w)*(x-xd)-L*y;
            dy = L*x;
        case 4
            % primal-dual (general extra with damping term) (Jakovetic 2019)
            a = 3;
            dx = -L*x-a*diag(w)*(x-xd)-L*y;
            dy = L*(x+dx);
        case 5
            % gradient tracking (time-invarant, y0=x0-xd0) (Qu 2018)
            if t==0
               y = diag(w)*(x-xd); 
            end
            a = 3;
            dx = -L*x-a*y;
            dy = -L*y+diag(w)*dx;
        case 6
            % gradient tracking (without initial requirement)
            a = 3;
            dx = -L*x-a*y;
            dy = -L*y+diag(w)*dx-(y-diag(w)*(x-xd))+z;
            dz = -L*y;
    end
    % update
    x = x+dx*dt;
    y = y+dy*dt;
    z = z+dz*dt;
    xd = xd+dxd*dt;
    x_opt = sum(w)^-1*sum(diag(w)*xd);
    % save
    xdata(:,:,loop) = x;
    ydata(:,:,loop) = y;
    % animate
    set(fxd,'xdata',xd(:,1),'ydata',xd(:,2));
    set(fxo,'xdata',x_opt(:,1),'ydata',x_opt(:,2));
    set(fx,'xdata',x(:,1),'ydata',x(:,2));
    set(fy,'xdata',y(:,1),'ydata',y(:,2));
    drawnow
end
