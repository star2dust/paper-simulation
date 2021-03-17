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
jacob_dot = @(joint,dot_joint) robot_desired.getJacobDot(robot_desired.link,joint,dot_joint);
hinge = @(joint,j) robot_desired.getHinge(robot_desired.link,joint,j,'b');
nabla = @(joint,j) robot_desired.getNablaHinge(robot_desired.link,joint,j,'b');
% initials
pb = [2,0]; 
pe_ref = [-1,-1]; 
th = robot_desired.ikine2d(pe_ref,[pb,pi])-[pi,zeros(1,link_num-1)];
dot_th = zeros(size(th));
pe = pb+fkine(th);
% references
dot_pe_ref = zeros(size(pe_ref));
angle_ref = -pi*3/4; angle_safe = 0.9*pi;
th_ref = joint_ref_func(angle_ref,3/4);
th_ref = suitangle(th,th_ref);
dot_th_ref = zeros(size(th_ref));
pb_ref = pe_ref-fkine(th_ref);
% region
V = [-1,4;6,-5];
kb = [V(:,1),ones(size(V,1),1)]^-1*V(:,2);
Areg = [-kb(1),1]/norm([-kb(1),1]);
breg = kb(2)/norm([-kb(1),1]);
k = 1;
dk = 0;
% figure
figure
gca; hold on
robot_desired.plot(th,[pb,0]);
fpbr = plot3(pb_ref(1),pb_ref(2),2,'rs');
fpe = plot3(pe_ref(1),pe_ref(2),2,'co');
plot(V(:,1), V(:,2), 'ro-', 'LineWidth', 2);
axis([-1 9 -5 5]);

% simulation
dt = 0.01;
T = 5;
loop = 0;
color_list = ['b','g','m','r','c'];
for t=0:dt:T
    loop = loop+1;
    % optmization
    dot_pe_ref = unit(diff(V))*rot2(pi/2)'/2;
    jac_ref = jacob(th_ref);
    fk_ref = fkine(th_ref);
    jac_ref_dot = jacob_dot(th_ref,dot_th_ref);
    fk = fkine(th);
    jac = jacob(th);
    % cost function
    Acost = [eye(2),jac_ref;zeros(link_num,2),eye(link_num)];
    bcost = [jac_ref*th_ref'-fk_ref'+pe_ref';th_ref'];
    dAcost = [zeros(2),jac_ref_dot;zeros(link_num,2),zeros(link_num)];
    dbcost = [jac_ref_dot*th_ref'+dot_pe_ref';dot_th_ref'];
    [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost([pb,th],t,Acost,bcost,dAcost,dbcost);
    % barrier
    Ath = [0,1,0;0,-1,0;0,0,1;0,0,-1];
    bth = [pi*0.9;0;0;pi*0.9];
    Abarr = blkdiag(Areg,Ath);
    bbarr = [breg;bth];
    [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier([pb,th],t,Abarr,bbarr);
    % check bar
    if imag(bar)
        bar_nabla = bar_nabla*0;
        bar_nabla_dot = bar_nabla_dot*0;
        bar_hessian = bar_hessian*0;
    end
    phi = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
    hessian_inv = (cost_hessian+bar_hessian)^-1;
    unom(1,:) = -hessian_inv*phi;
%     dot_pb = unom(1:2);
%     dot_th = unom(3:end);
    % extended K-class function
    akcf = 1;
    ekcf = @(h) akcf*h;
    % cbf - quadratic program
    Aqp = [Areg,zeros(size(Areg,1),link_num),ekcf(Areg*pb'-breg)];
    bqp = zeros(size(Areg,1),1);
    for j=1:link_num
        Aqp = [Aqp;Areg,Areg*nabla(th,j),ekcf(Areg*(pb'+hinge(th,j)')-breg)];
        bqp = [bqp;zeros(size(Areg,1),1)];
    end
    Aqp = [Aqp;zeros(size(Ath,1),2),Ath,ekcf(Ath*th'-bth);zeros(1,2+link_num),-1];
    bqp = [bqp;zeros(size(Ath,1),1);ekcf(k)];
    % end constraint
    rad = 0.1;
    [Acirc,bcirc] = circzcbf(pe,pe_ref,rad,dot_pe_ref,0);
    Aqp = [Aqp;Acirc*[eye(2),jac],ekcf((pe-pe_ref)*(pe-pe_ref)'-rad^2)];
    bqp = [bqp;bcirc+ekcf((pe-pe_ref)*(pe-pe_ref)'-rad^2)];
    [uqp(1,:),f,is_solved] = quadprog(eye(link_num+3),[-unom';0],Aqp,bqp);
    dot_pb = uqp(1:2);
    dot_th = uqp(3:link_num+2);
    dk = uqp(end);
    % update
    k = k+dt*dk;
    pb = pb+dt*dot_pb;
    th = th+dt*dot_th;
    pe = pb+fk;
    pe_ref = pe_ref+dt*dot_pe_ref;
    pb_ref = pe_ref-fkine(th_ref);
    % plot
    robot_desired.animate(th,[pb,0]);
    set(fpe,'xdata',pe_ref(1),'ydata',pe_ref(2));
    set(fpbr,'xdata',pb_ref(1),'ydata',pb_ref(2));
    hold off
    drawnow
end


