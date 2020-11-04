close all
clear

pos_rob = [-0.5,-0.5;0.5,0.5];
pos_ref =  [1.1,1;-1.1,-1];
pos_circ = [-0.5,-0.5;0.5,0.5]/2;

% circular constraint sets
zbf_circ = @(i,pos_rob) 1-(pos_rob(i,:)-pos_circ(i,:))*(pos_rob(i,:)-pos_circ(i,:))';
% nominal control
knom = 1;
unom_rob = @(i,pos_rob) -knom*(pos_rob(i,:)-pos_ref(i,:));
% bounded distance costraints
ld = 0.5; ud = 1;
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

% simulation
dt = 0.01;
T = 10;
loop = 0;
color_list = ['b','g','m','r','c'];
for t=0:dt:T
    loop = loop+1;
    % control barrier function - quadratic program
    for i=1:2
        fqp = -unom_rob(i,pos_rob)';
        aqp = cbfa_circ(i,pos_rob);
        bqp = cbfb_circ(i,pos_rob);
        for j=1:2
            if i~=j
                aqp = [aqp;cbfa_lb(i,j,pos_rob)];
                bqp = [bqp;cbfb_lb(i,j,pos_rob)];
                aqp = [aqp;cbfa_ub(i,j,pos_rob)];
                bqp = [bqp;cbfb_ub(i,j,pos_rob)];
            end
        end
        [uqp,~,is_solved] = quadprog(eye(2),fqp,aqp,bqp);
        uqp_rob(i,:) = uqp';
    end
    % update
    pos_rob = pos_rob+dt*uqp_rob;
    % plot
    for i=1:2
        plot(pos_rob(i,1),pos_rob(i,2),[color_list(i) 'o']); hold on
        circle_(pos_circ(i,:),1,'color',color_list(i));
        plot(pos_ref(i,1),pos_ref(i,2),[color_list(i) 'd']);
        quiver(pos_rob(i,1),pos_rob(i,2),uqp_rob(i,1),uqp_rob(i,2))
    end
    hold off
    drawnow
end


