close all
clear

robot_num = 5;
% reference
angle_central = 2*pi/robot_num;
if ~mod(robot_num,2)
    pos_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1]) robot_num/2]*angle_central)']/2;
else
    pos_ref = [cos([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)',...
        sin([0 kron(1:(robot_num-1)/2,[1,-1])]*angle_central)']/2;
end
% topology
graph = selectTopology(robot_num,pos_ref);
% initial position
pos_rob = pos_ref*rot2(2*pi/3);
% initial control
u_rob = zeros(size(pos_rob));
% reference control
u_ref_func = @(t) pos_ref./normby(pos_ref,1)*sin(pi*t)/2.*(-1).^(0:robot_num-1)';
u_ref = u_ref_func(0);
% plot
color_list = ['b','g','m','r','c'];
for i=1:robot_num
    hg_pos_rob(i) = plot(pos_rob(i,1),pos_rob(i,2),[color_list(i) 'o']); hold on
    pos_circ = circle_(pos_ref(i,:),1);
    hg_pos_circ(i) = plot(pos_circ(:,1),pos_circ(:,2),color_list(i));
    hg_pos_ref(i) = plot(pos_ref(i,1),pos_ref(i,2),[color_list(i) 'd']); 
%     hg_u_rob(i) = quiver(pos_rob(i,1),pos_rob(i,2),u_rob(i,1),u_rob(i,2),'color',color_list(i));
    hg_u_ref(i) = quiver(pos_ref(i,1),pos_ref(i,2),u_ref(i,1),u_ref(i,2),'color',color_list(i),'LineStyle', '--');
    axis([-2 2 -2 2])
end
hold off

% functions
a1 = 1; a2 = 1; rho = a1*exp(a2*0); cost_hessian = 2*eye(2);
cost_ref = @(i,pos_rob,pos_ref) (pos_rob(i,:)-pos_ref(i,:))*(pos_rob(i,:)-pos_ref(i,:))';
cost_ref_dot = @(i,pos_rob,u_ref) -2*(pos_rob(i,:)-pos_ref(i,:))*u_ref(i,:)';
cost_ref_nabla = @(i,pos_rob,pos_ref) 2*(pos_rob(i,:)-pos_ref(i,:))';
cost_ref_nabla_dot = @(i,u_ref) -2*u_ref(i,:)';
cost_penalty = @(i,pos_rob,rho) cost_ref(i,pos_rob,pos_ref)-(1/rho)*log(1-rho*(cost_ref(i,pos_rob,pos_ref)-1));
cost_penalty_nabla = @(i,pos_rob,pos_ref,rho) cost_ref_nabla(i,pos_rob,pos_ref)*(1+1/(1-rho*(cost_ref(i,pos_rob,pos_ref)-1)));
cost_penalty_nabla_dot = @(i,pos_rob,pos_ref,rho) cost_ref_nabla_dot(i,u_ref)*(1+1/(1-rho*(cost_ref(i,pos_rob,pos_ref)-1)))+...
    rho*(a2*(cost_ref(i,pos_rob,pos_ref)-1)+cost_ref_dot(i,pos_rob,u_ref))/(1-rho*(cost_ref(i,pos_rob,pos_ref)-1))^2;
cost_penalty_hessian = @(i,pos_rob,pos_ref,rho) cost_hessian*(1+1/(1-rho*(cost_ref(i,pos_rob,pos_ref)-1)))+...
    rho*cost_ref_nabla(i,pos_rob,pos_ref)*cost_ref_nabla(i,pos_rob,pos_ref)'/(1-rho*(cost_ref(i,pos_rob,pos_ref)-1));
% test error
cost_ref(i,pos_rob,pos_ref);
cost_ref_dot(i,pos_rob,u_ref);
cost_ref_nabla(i,pos_rob,pos_ref);
cost_ref_nabla_dot(i,u_ref);
cost_penalty(i,pos_rob,rho);
cost_penalty_nabla(i,pos_rob,pos_ref,rho);
cost_penalty_nabla_dot(i,pos_rob,pos_ref,rho);
cost_penalty_hessian(i,pos_rob,pos_ref,rho);
disp('All functions are correct.');

% simulation
pos_data = pos_rob;
dt = 0.005;
T = 10;
loop = 0;
playspeed = 4;
video_on = false;
for t=0:dt:T
    loop = loop+1;
    % control
    u_ref = u_ref_func(t);
    beta = 1; hessian_inv = []; u_rob_tp = u_rob';
    for i=1:robot_num
       phi(:,i) = -cost_penalty_hessian(i,pos_rob,pos_ref,rho)^-1*...
           (cost_penalty_nabla(i,pos_rob,pos_ref,rho)+cost_penalty_nabla_dot(i,pos_rob,pos_ref,rho));
       hessian_inv = blkdiag(hessian_inv,cost_penalty_hessian(i,pos_rob,pos_ref,rho)^-1);
    end
%     sign_rob = (graph.incidence*sign(graph.incidence'*pos_rob))';
    sign_rob = (graph.stress*sign(graph.stress*pos_rob))';
    u_rob_tp(:) = -beta*hessian_inv*sign_rob(:)+phi(:);
    u_rob = u_rob_tp';
    % update
    pos_ref = pos_ref+dt*u_ref;
    pos_rob = pos_rob+dt*u_rob;
    pos_data(:,:,loop) = pos_rob;
    % plot
    for i=1:robot_num
        set(hg_pos_rob(i),'xdata',pos_rob(i,1),'ydata',pos_rob(i,2)); 
        pos_circ = circle_(pos_ref(i,:),1);
        set(hg_pos_circ(i),'xdata',pos_circ(:,1),'ydata',pos_circ(:,2));
        set(hg_pos_ref(i),'xdata',pos_ref(i,1),'ydata',pos_ref(i,2));
%         set(hg_u_rob(i),'xdata',pos_rob(i,1),'ydata',pos_rob(i,2),...
%             'udata',u_rob(i,1),'vdata',u_rob(i,2));
        set(hg_u_ref(i),'xdata',pos_ref(i,1),'ydata',pos_ref(i,2),...
            'udata',u_ref(i,1),'vdata',u_ref(i,2));
        axis([-2 2 -2 2])
    end
    % video
    if mod(loop,playspeed)==0&&video_on
        frame(loop/playspeed) = getframe(gcf);
    end
    drawnow
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