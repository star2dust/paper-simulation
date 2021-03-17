close all
clear

robot_num = 5;
% reference
angle_central = 2*pi/robot_num;
if ~mod(robot_num,2)
    vert_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)'];
else
    vert_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)'];
end
% end and base
end_siz = 2;
base_siz = 5;
end_ref = vert_ref*end_siz;
base_ref = vert_ref*base_siz;
% topology
base_lnum = 3; end_l_num = 3;
base_lind = element(randperm(robot_num),1:3);
end_graph = selectTopology(robot_num,end_ref);
base_graph = selectTopology(robot_num+base_lnum,[end_ref(base_lind,:);base_ref]);
% planning
qvia = [-10,-10,0;
    5,-10,-pi/4;
    -15,10,-pi/2;
    -15,-15,-pi/2];
[qr,dqr,ddqr,tr] = mstraj_(qvia,ones(1,size(qvia,2)),0.1,2);
% region
A = [0.7071,0.7071;
    -1,0;
    0,-1];
b = [0;20;20];
% initial position
for i=1:robot_num
    end_pos(i,:) = rot2transl(qr(1,3),end_ref(i,:))'+qr(1,1:2)+rand(1,2);
    base_pos(i,:) = rot2transl(qr(1,3),base_ref(i,:))'+qr(1,1:2)+rand(1,2);
end
% initial control
u_end = zeros(size(end_pos));
u_base = zeros(size(base_pos));
% plot
color_list = ['b','g','m','r','c'];
gca; hold on;
for i=1:robot_num
    if ~isempty(A)
        V = lcon2vert(A, b);
        k = convhull(V(:,1), V(:,2));
        hg_region(i) = plot(V(k,1), V(k,2), 'ro-', 'LineWidth', 2);
    end
    hg_end_pos(i) = plot(end_pos(i,1),end_pos(i,2),[color_list(i) 'o']);
    hg_end_ref(i) = plot(end_ref(i,1),end_ref(i,2),[color_list(i) 'd']); 
    hg_base_pos(i) = plot(base_pos(i,1),base_pos(i,2),[color_list(i) 's']);
    axis([-20 20 -20 20])
end
hold off

% simulation
pos_data = end_pos;
dt = 0.01;
T = 50;
loop = 0;
playspeed = 10;
video_on = false;
for t=0:dt:T
    loop = loop+1;
    % control
    u_ref = reshape(interp1(tr,dxr,t)',[2,robot_num])';
    hessian_inv = []; u_rob_tp = u_end';
    for i=1:robot_num
        [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost(end_pos(i,:),t,eye(2),end_ref(i,:)',zeros(2),u_ref(i,:)');
        [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier(end_pos(i,:),t,A,b);
        phi(:,i) = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
        hessian_inv = blkdiag(hessian_inv,(cost_hessian+bar_hessian)^-1);
    end
%     sign_rob = (graph.incidence*sign(graph.incidence'*pos_rob))';
    sign_rob = (graph.stress*sign(graph.stress*end_pos))';
    beta = norm(phi(:))+1;
    u_rob_tp(:) = -hessian_inv*(beta*sign_rob(:)+phi(:));
    u_end = u_rob_tp';
    % update
    end_ref = end_ref+dt*u_ref;
    end_pos = end_pos+dt*u_end;
    pos_data(:,:,loop) = end_pos;
    % plot
    for i=1:robot_num
        set(hg_end_pos(i),'xdata',end_pos(i,1),'ydata',end_pos(i,2)); 
        set(hg_end_ref(i),'xdata',end_ref(i,1),'ydata',end_ref(i,2));
%         set(hg_u_rob(i),'xdata',pos_rob(i,1),'ydata',pos_rob(i,2),...
%             'udata',u_rob(i,1),'vdata',u_rob(i,2));
        set(hg_u_ref(i),'xdata',end_ref(i,1),'ydata',end_ref(i,2),...
            'udata',u_ref(i,1),'vdata',u_ref(i,2));
        axis([-20 20 -20 20])
    end
    % video
    if mod(loop,playspeed)==0
        frame(loop/playspeed) = getframe(gcf);
        drawnow
    end
end
% write video
if video_on 
    savevideo('video',frame);
end
figure 
t_data = 0:dt:T;
xpos_data = squeeze(pos_data(:,1,:));
ypos_data = squeeze(pos_data(:,2,:));
plot3(kron(ones(robot_num,1),t_data)',xpos_data',ypos_data');
xlabel('time/s');ylabel('x/m');zlabel('y/m');grid
figure
plot(kron(ones(robot_num,1),t_data)',xpos_data');
xlabel('time/s');ylabel('x/m');grid
figure
plot(kron(ones(robot_num,1),t_data)',ypos_data');
xlabel('time/s');ylabel('y/m');grid