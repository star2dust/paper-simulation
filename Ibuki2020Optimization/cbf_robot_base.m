close all
clear

radius = 0.8;
altitude = 1;
height = 1;
link_list = [1 2 1];
link_num = length(link_list);
angle_safety = 0.9*pi;
[joint_ref_func,robot_desired] = Mascot.jointRef(radius,altitude,height,link_list,angle_safety);
fkine = @(joint) robot_desired.getFkine(robot_desired.link,joint);
jacob = @(joint) robot_desired.getJacob(robot_desired.link,joint);

pe_ref = [0,0];
th_ref = joint_ref_func(-pi*3/4,3/4);
pb_ref = pe_ref-fkine(th_ref);
% robot_desired.plot(th_ref,[pb_ref,0]);

pb = [2,0];
th = robot_desired.ikine2d([0,0],[pb,pi])+[pi,zeros(1,link_num-1)];
th_ref = suitangle(th,th_ref);
% robot_desired.plot(th_ref,[pb_ref,0]);

V = [-1,4;6,-5];
kb = [V(:,1),ones(size(V,1),1)]^-1*V(:,2);
Abar = [-kb(1),1]/norm([-kb(1),1]);
bbar = kb(2)/norm([-kb(1),1]);

figure
gca; hold on
robot_desired.plot(th,[pb,0]);
fpe = plot(pe_ref(1),pe_ref(2),'co');
plot(V(:,1), V(:,2), 'ro-', 'LineWidth', 2);
axis([-1 9 -5 5]);

% simulation
dt = 0.01;
T = 10;
loop = 0;
color_list = ['b','g','m','r','c'];
for t=0:dt:T
    loop = loop+1;
    % optmization
    dot_pe_ref = unit(diff(V))*rot2(pi/2)'/10;
    jacob_hat = jacob(th_ref);
    fkine_hat = fkine(th_ref);
    Acost = [eye(2),jacob_hat;zeros(link_num,2),eye(link_num)];
    bcost = [-fkine_hat'+jacob_hat*th_ref'+pe_ref';th_ref'];
    dAcost = [zeros(2,2+link_num);zeros(link_num,2+link_num)];
    dbcost = [dot_pe_ref';zeros(size(th_ref))'];
    [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost([pb,th],t,Acost,bcost,dAcost,dbcost);
    [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier(pb,t,Abar,bbar);
    if imag(bar)
        bar_nabla = bar_nabla*0;
        bar_nabla_dot = bar_nabla_dot*0;
        bar_hessian = bar_hessian*0;
    end
    phi = cost_nabla+[bar_nabla;zeros(link_num,1)]+cost_nabla_dot+[bar_nabla_dot;zeros(link_num,1)];
    hessian_inv = (cost_hessian+blkdiag(bar_hessian,zeros(link_num)))^-1;
    % algorithm
    uq = -hessian_inv*phi;
    % update
    pb = pb+dt*uq(1:2)';
    th = th+dt*uq(3:end)';
    pe_ref = pe_ref+dt*dot_pe_ref;
    % plot
    robot_desired.animate(th,[pb,0]);
    set(fpe,'xdata',pe_ref(1),'ydata',pe_ref(2));
    hold off
    drawnow
end


