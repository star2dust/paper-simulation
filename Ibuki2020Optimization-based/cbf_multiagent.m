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
% circle centers and initials
pos_circ = pos_ref;
% pos_rob = pos_circ.*rand(robot_num,2)*2;
pos_rob = pos_ref*rot2(2*pi/3);
color_list = ['b','g','m','r','c'];
for i=1:robot_num
    plot(pos_rob(i,1),pos_rob(i,2),[color_list(i) 'o']); hold on
    circle_(pos_circ(i,:),1,'color',color_list(i));
    plot(pos_ref(i,1),pos_ref(i,2),[color_list(i) 'd']); 
end
hold off
% initial control
uqp_rob = zeros(size(pos_rob));

% circular constraint sets
zbf_circ = @(i,pos_rob) 1-(pos_rob(i,:)-pos_circ(i,:))*(pos_rob(i,:)-pos_circ(i,:))';
% nominal control
knom = 2;
unom_rob = @(i,pos_rob) -knom*(pos_rob(i,:)-pos_ref(i,:));
% bounded distance costraints
ld = 0.3; ud = 0.8;
zbf_lb = @(i,j,pos_rob) (pos_rob(i,:)-pos_rob(j,:))*(pos_rob(i,:)-pos_rob(j,:))'-ld^2;
zbf_ub = @(i,j,pos_rob) ud^2-(pos_rob(i,:)-pos_rob(j,:))*(pos_rob(i,:)-pos_rob(j,:))';
% extended K-class function
akcf = 1;
ekcf = @(h) akcf*h;
% control barrier function (less or equal)
cbfa_circ = @(i,pos_rob) pos_rob(i,:)-pos_circ(i,:);
cbfb_circ = @(i,pos_rob) ekcf(zbf_circ(i,pos_rob));
cbfa_lb = @(i,j,pos_rob) -(pos_rob(i,:)-pos_rob(j,:));
cbfb_lb = @(i,j,pos_rob) ekcf(zbf_lb(i,j,pos_rob));
cbfa_ub = @(i,j,pos_rob) -cbfa_lb(i,j,pos_rob);
cbfb_ub = @(i,j,pos_rob) ekcf(zbf_ub(i,j,pos_rob));
% another bounded distance costraints
ld = 0.3; ud = 0.8; 
zbf_bd = @(i,j,pos_rob) -((pos_rob(i,:)-pos_rob(j,:))*(pos_rob(i,:)-pos_rob(j,:))'-ld^2)...
    *((pos_rob(i,:)-pos_rob(j,:))*(pos_rob(i,:)-pos_rob(j,:))'-ud^2);
% another control barrier function
cbfa_bd = @(i,j,pos_rob) (2*(pos_rob(i,:)-pos_rob(j,:))*(pos_rob(i,:)-pos_rob(j,:))'-ld^2-ud^2)*(pos_rob(i,:)-pos_rob(j,:));
cbfb_bd = @(i,j,pos_rob,uqp_rob) ekcf(zbf_bd(i,j,pos_rob))+cbfa_bd(i,j,pos_rob)*uqp_rob(j,:)';
% cbfb_bd = @(i,j,pos_rob) ekcf(zbf_bd(i,j,pos_rob));

cbf_type = 2;
pos_data = pos_rob;
dt = 0.01;
T = 10;
loop = 0;
for t=0:dt:T
    loop = loop+1;
    % control barrier function - quadratic program
    for i=1:robot_num
        fqp = -unom_rob(i,pos_rob)';
        aqp = cbfa_circ(i,pos_rob);
        bqp = cbfb_circ(i,pos_rob);
        for j=graph.edge_set(2,graph.edge_set(1,:)==i)       
            if cbf_type==1
                aqp = [aqp;cbfa_lb(i,j,pos_rob)];
                aqp = [aqp;cbfa_ub(i,j,pos_rob)];
                bqp = [bqp;cbfb_lb(i,j,pos_rob)];
                bqp = [bqp;cbfb_ub(i,j,pos_rob)];
            else
                aqp = [aqp;cbfa_bd(i,j,pos_rob)];
                bqp = [bqp;cbfb_bd(i,j,pos_rob,uqp_rob)];
%                 bqp = [bqp;cbfb_bd(i,j,pos_rob)];
            end
        end
        [uqp,~,is_solved] = quadprog(eye(2),fqp,aqp,bqp);
        if is_solved
            uqp_rob(i,:) = uqp';
        else
            uqp_rob(i,:) = [0,0];
        end
    end
    % update
    pos_rob = pos_rob+dt*uqp_rob;
    pos_data(:,:,loop) = pos_rob;
    % plot
    for i=1:robot_num
        plot(pos_rob(i,1),pos_rob(i,2),[color_list(i) 'o']); hold on
        circle_(pos_circ(i,:),1,'color',color_list(i));
        plot(pos_ref(i,1),pos_ref(i,2),[color_list(i) 'd']);
        quiver(pos_rob(i,1),pos_rob(i,2),uqp_rob(i,1)/knom,uqp_rob(i,2)/knom,'color',color_list(i));
    end
    hold off
    drawnow
end
