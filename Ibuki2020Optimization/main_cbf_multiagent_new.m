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
graph = bigraph(robot_num,pos_ref);
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
% reference distance
dis_ref = distances(graph,pos_ref);
ld = dis_ref-0.2; ud = dis_ref+0.1;

% circular constraint sets
zbf_circ = @(i,pos_rob) 1-(pos_rob(i,:)-pos_circ(i,:))*(pos_rob(i,:)-pos_circ(i,:))';
% nominal control
knom = 2;
unom_rob = @(i,pos_rob) -knom*(pos_rob(i,:)-pos_ref(i,:));
% extended K-class function
akcf = 1;
ekcf = @(h) akcf*h;
% control barrier function (less or equal)
cbfa_circ = @(i,pos_rob) pos_rob(i,:)-pos_circ(i,:);
cbfb_circ = @(i,pos_rob) ekcf(zbf_circ(i,pos_rob));

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
        [aqp,bqp] = bdzcbf(graph,i,pos_rob,ld,ud,aqp,bqp,uqp_rob);
        [uqp,~,is_solved] = quadprog(eye(2),fqp,aqp,bqp);
        if is_solved>0
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
