close all
clear

df = @(x) [x(1)+1/4;2*x(2)+1/3;2*x(3)/5+1/6;x(4)/2+1];
at = @(t) 1/t;
lim = [-3,-2,0,-4;
    3,1,2,4];
x = [5,-4,3,-2]';
alpha = 1;
beta = 0.5;
y = [0,0,0,0]';
% y = rand(4,1);

A = [0,0,0,1;
    1,0,0,0;
    0,1,0,0;
    0,0,1,0];
% A = [0,1,0,1;
%     1,0,1,0;
%     0,1,0,1;
%     1,0,1,0];
D = diag(sum(A,2));
L = D-A;


T = 40;
dt = 0.001;
c = 0;
for t=0:dt:T
    c = c+1;
    % primal-dual for undirected graph
%     dx = convproj(x-alpha*df(x)-beta*L*y,[],[],lim)'-x;
%     dy = beta*L*x;
    % wang2018(wrong)
%     dx = convproj(x-alpha*df(x),[],[],lim)'-x+beta*L*y;
%     dy = -beta*L*x;
    % kia2015
    dx = -alpha*df(x)-beta*L*x-y;
    dy = alpha*beta*L*x;
    % wang2018+kia2015(wrong)
%     dx = convproj(x-alpha*df(x),[],[],lim)'-x-beta*L*x-y;
%     dy = beta*L*x;
    % kia2015+projection
%     dx = convproj(x-alpha*df(x)-beta*L*x-y,[],[],lim)'-x;
%     dy = alpha*beta*L*x;
    
    x = x+dx*dt;
    y = y+dy*dt;
    
    xx(:,c) = x;
    yy(:,c) = y;
    tt(c) = t;
end
figure
color = ['b' 'g' 'm' 'r'];
plot(tt,xx(1,:),color(1),tt,xx(2,:),color(2),tt,xx(3,:),color(3),tt,xx(4,:),color(4));
hold on
plot(tt,lim(1,1)*ones(size(tt)),color(1),tt,lim(2,1)*ones(size(tt)),color(1));
plot(tt,lim(1,2)*ones(size(tt)),color(2),tt,lim(2,2)*ones(size(tt)),color(2));
plot(tt,lim(1,3)*ones(size(tt)),color(3),tt,lim(2,3)*ones(size(tt)),color(3));
plot(tt,lim(1,4)*ones(size(tt)),color(4),tt,lim(2,4)*ones(size(tt)),color(4));
figure
plot(tt,yy(1,:),color(1),tt,yy(2,:),color(2),tt,yy(3,:),color(3),tt,yy(4,:),color(4));
