close all
clear

robot_num = 5;
% reference
angle_central = 2*pi/robot_num;
if ~mod(robot_num,2)
    pos_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)']*5;
else
    pos_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)']*5;
end
% topology
graph = bigraph(robot_num,pos_ref);
% planning
qvia = [-10,-10,0;
    5,-10,-pi/4;
    -15,10,-pi/2;
    -15,-15,-pi/2];
pvia = zeros(size(qvia,1),robot_num*2);
for j=1:size(qvia,1)
    for i=1:robot_num
        pvia(j,2*(i-1)+1:2*i) = pos_ref(i,:)*rot2(qvia(j,3))'+qvia(j,1:2);
    end
end
% generate trajectory
[xr,dxr,ddxr,tr] = mstraj_(pvia,ones(1,size(pvia,2)),0.1,2);
pos_ref = reshape(xr(1,:)',[2,robot_num])';
u_ref = reshape(dxr(1,:)',[2,robot_num])';
% region
Abar = [0.7071,0.7071;
    -1,0;
    0,-1];
bbar = [0;20;20];
% initial position
pos_rob = pos_ref*rand(2)+rand(size(pos_ref));
% initial control
u_rob = zeros(size(pos_rob));
% reference distance
dis_ref = distances(graph,pos_ref);
ld = dis_ref-0.2; ud = dis_ref+0.2;
% plot
color_list = ['b','g','m','r','c'];
gca; hold on;
for i=1:robot_num
    if ~isempty(Abar)
        V = lcon2vert(Abar, bbar);
        k = convhull(V(:,1), V(:,2));
        hg_region(i) = plot(V(k,1), V(k,2), 'ro-', 'LineWidth', 2);
    end
    hg_pos_rob(i) = plot(pos_rob(i,1),pos_rob(i,2),[color_list(i) 'o']);
    hg_pos_ref(i) = plot(pos_ref(i,1),pos_ref(i,2),[color_list(i) 'd']); 
%     hg_u_rob(i) = quiver(pos_rob(i,1),pos_rob(i,2),u_rob(i,1),u_rob(i,2),'color',color_list(i));
    hg_u_ref(i) = quiver(pos_ref(i,1),pos_ref(i,2),u_ref(i,1),u_ref(i,2),'color',color_list(i),'LineStyle', '--');
    axis([-20 20 -20 20])
end
hold off

% simulation
pos_data = pos_rob;
dt = 0.01;
T = 50;
loop = 0;
playspeed = 2;
video_on = true;
for t=0:dt:T
    loop = loop+1;
    % control
    u_ref = reshape(interp1(tr,dxr,t)',[2,robot_num])';
    hessian_inv = []; u_rob_tp = u_rob';
    for i=1:robot_num
        [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost(pos_rob(i,:),t,eye(2),pos_ref(i,:)',zeros(2),u_ref(i,:)');
        [bar(i),bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier(pos_rob(i,:),t,Abar,bbar);
        % check bar
        if imag(bar(i))
            bar_nabla = bar_nabla*0;
            bar_nabla_dot = bar_nabla_dot*0;
            bar_hessian = bar_hessian*0;
        end
        phi(:,i) = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
        hessian_inv = blkdiag(hessian_inv,(cost_hessian+bar_hessian)^-1);
    end
%     sign_rob = (graph.incidence*sign(graph.incidence'*pos_rob))';
    sign_rob = (graph.stress*sign(graph.stress*pos_rob))';
    beta = max(normby(phi,2));
    u_rob_tp(:) = -hessian_inv*(beta*sign_rob(:)+phi(:));
    u_rob = u_rob_tp';
    % cbf
    for i=1:robot_num
        % bound cbf [aqp,bqp] = bdzcbf(graph,i,pos_rob,ld,ud,[],[],u_rob);
        [aqp1,bqp1] = bdzcbf(graph,i,pos_rob,ld,ud);
        % extended K-class function
        akcf = 1;
        ekcf = @(h) akcf*h;
        % region cbf
        aqp2 = Abar;
        bqp2 = -ekcf(Abar*pos_rob(i,:)'-bbar);
        [u_rob_qp(:,i),~,is_solved] = quadprog(eye(2),-u_rob_tp(:,i),[aqp1;aqp2],[bqp1;bqp2]);
        if is_solved<=0
            u_rob_qp(:,i) = u_rob_tp(:,i);
        end
    end
    u_rob = u_rob_qp';
    % update
    pos_ref = pos_ref+dt*u_ref;
    pos_rob = pos_rob+dt*u_rob;
    pos_data(:,:,loop) = pos_rob;
    % plot
    for i=1:robot_num
        set(hg_pos_rob(i),'xdata',pos_rob(i,1),'ydata',pos_rob(i,2)); 
        set(hg_pos_ref(i),'xdata',pos_ref(i,1),'ydata',pos_ref(i,2));
%         set(hg_u_rob(i),'xdata',pos_rob(i,1),'ydata',pos_rob(i,2),...
%             'udata',u_rob(i,1),'vdata',u_rob(i,2));
        set(hg_u_ref(i),'xdata',pos_ref(i,1),'ydata',pos_ref(i,2),...
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
    savevideo('affopt_nocbf',frame);
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