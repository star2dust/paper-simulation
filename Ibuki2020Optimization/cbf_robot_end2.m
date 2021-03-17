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
hinge = @(joint,j) robot_desired.getHinge(robot_desired.link,joint,j,'e');
nabla = @(joint,j) robot_desired.getNablaHinge(robot_desired.link,joint,j,'e');
mu = @(joint) robot_desired.getMu(robot_desired.link,joint);
% initials
pb = [2,0]; pb_hat = pb;
pe_ref = [-1,-1]; 
th = robot_desired.ikine2d(pe_ref,[pb,pi])-[pi,zeros(1,link_num-1)];
dot_th = zeros(size(th));
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
Abar = [-kb(1),1]/norm([-kb(1),1]);
bbar = kb(2)/norm([-kb(1),1]);
% figure
figure
gca; hold on
robot_desired.plot(th,[pb,0]);
fpbr = plot3(pb_ref(1),pb_ref(2),2,'rs');
fpe = plot3(pe_ref(1),pe_ref(2),2,'co');
fpbh = plot3(pb_hat(1),pb_hat(2),2,'cs');
plot(V(:,1), V(:,2), 'ro-', 'LineWidth', 2);
axis([-1 9 -5 5]);


% simulation
dt = 0.01;
T = 6;
loop = 0;
color_list = ['b','g','m','r','c'];
for t=0:dt:T
    loop = loop+1;
    % kinematics
    dot_pe_ref = unit(diff(V))*rot2(pi/2)'/2;
    jac_ref = jacob(th_ref);
    fk_ref = fkine(th_ref);
    fk = fkine(th);
    jac = jacob(th);
    % pb_hat optimization
    [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost(pb_hat,t,eye(2),pe_ref-fk_ref,zeros(2),dot_pe_ref-dot_th_ref*jac_ref');
    [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier(pb_hat,t,Abar,bbar);
    cost+bar
    if imag(bar)
        bar_nabla = bar_nabla*0;
        bar_nabla_dot = bar_nabla_dot*0;
        bar_hessian = bar_hessian*0;
    end
    phi = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
    hessian_inv = (cost_hessian+bar_hessian)^-1;
    dot_pb_hat(1,:) = -hessian_inv*phi;
    % th optimization
    dot_pbe_hat = dot_pe_ref-dot_pb_hat;
    pbe_hat = pe_ref-pb_hat;
    dthe = dot_pbe_hat-(fk-pbe_hat);
    [~,dmu(1,:)] = mu(th);
%     dmu = -(th-th_ref).*[0,ones(1,link_num-1)];
    dth_nom(1,:) = pinv(jac)*dthe'+(eye(link_num)-pinv(jac)*jac)*dmu';
%     dot_th = dth_nom;
    % cbf
    % extended K-class function
    akcf = 1;
    ekcf = @(h) akcf*h;
    % quadratic program
    aqp = -Abar*jac;
    bqp = -Abar*dot_pe_ref'-ekcf(Abar*(pe_ref'-fk')-bbar);
    for j=2:link_num
        aqp = [aqp;-Abar*nabla(th,j)];
        bqp = [bqp;-Abar*dot_pe_ref'-ekcf(Abar*(pe_ref'-hinge(th,j)')-bbar)];
    end
    [dot_th(1,:),f,is_solved] = quadprog(eye(link_num),-dth_nom',aqp,bqp);
    % update
    pb_hat = pb_hat+dt*dot_pb_hat;
    th = th+dt*dot_th;
    pe_ref = pe_ref+dt*dot_pe_ref;
    pb_ref = pe_ref-fkine(th_ref);
    % plot
    pb = pe_ref-fk;
    robot_desired.animate(th,[pb,0]);
    set(fpe,'xdata',pe_ref(1),'ydata',pe_ref(2));
    set(fpbh,'xdata',pb_hat(1),'ydata',pb_hat(2));
    set(fpbr,'xdata',pb_ref(1),'ydata',pb_ref(2));
    hold off
    drawnow
end


