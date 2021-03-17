close all
clear

radius = 0.8;
altitude = 1;
height = 1;
link_list = [1 2 1];
link_num = length(link_list);
link_siz = sum(link_list);
angle_safety = 0.9*pi;
[joint_ref_func,robot_desired] = Mascot.jointRef(radius,altitude,height,link_list,angle_safety);
fkine = @(joint) robot_desired.getFkine(robot_desired.link,joint);
jacob = @(joint) robot_desired.getJacob(robot_desired.link,joint);
jacob_dot = @(joint,dot_joint) robot_desired.getJacobDot(robot_desired.link,joint,dot_joint);
hinge = @(joint,j) robot_desired.getHinge(robot_desired.link,joint,j,'e');
nabla = @(joint,j) robot_desired.getNablaHinge(robot_desired.link,joint,j,'e');
mu = @(joint) robot_desired.getMu(robot_desired.link,joint);
% initials
pb = [2,0]; 
pe_ref = [-1,-1]; 
th = robot_desired.ikine2d(pe_ref,[pb,pi])-[pi,zeros(1,link_num-1)];
dot_th = zeros(size(th));
r = norm(pb-pe_ref);
r_ref = link_siz*0.5;
dot_r = zeros(size(r));
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
Ath = [0,1,0;0,-1,0;0,0,1;0,0,-1];
bth = [pi*0.9;0;0;pi*0.9];
% figure
figure
gca; hold on
robot_desired.plot(th,[pb,0]);
fpbr = plot3(pb_ref(1),pb_ref(2),2,'rs');
fpe = plot3(pe_ref(1),pe_ref(2),2,'co');
xcirc = circle_(pe_ref,r);
fcirc = plot3(xcirc(:,1),xcirc(:,2),2*ones(size(xcirc,1),1),'g');
plot(V(:,1), V(:,2), 'ro-', 'LineWidth', 2);
axis([-1 9 -5 5]);


% simulation
dt = 0.01;
T = 5;
loop = 0;
color_list = ['b','g','m','r','c'];
video_on = false;
for t=0:dt:T
    loop = loop+1;
    % dot_pe_ref and dot_r
    dot_pe_ref = unit(diff(V))*rot2(pi/2)'/2;
    dot_r = -(r-r_ref);
    % kinematics
    jac_ref = jacob(th_ref);
    fk_ref = fkine(th_ref);
    jac_ref_dot = jacob_dot(th_ref,dot_th_ref);
    fk = fkine(th);
    jac = jacob(th);
    % cost function
    Acost = eye(2);
    bcost = -fk_ref'+pe_ref';
    dAcost = zeros(2);
    dbcost = jac_ref*dot_th_ref'+dot_pe_ref';
    [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost(pb',t,Acost,bcost,dAcost,dbcost);
    % barrier
    [bar1,bar1_nabla,bar1_nabla_dot,bar1_hessian] = logbarrier(pb,t,Areg,breg);
    [circ,circ_nabla,circ_nabla_dot,circ_hessian] = circbarrier(pb,t,pe_ref,r,dot_pe_ref,dot_r);
    bar = bar1+circ;
    bar_nabla = bar1_nabla+circ_nabla;
    bar_nabla_dot = bar1_nabla_dot+circ_nabla_dot;
    bar_hessian = bar1_hessian+circ_hessian;
    % cost+bar
    if imag(bar)
        bar_nabla = bar_nabla*0;
        bar_nabla_dot = bar_nabla_dot*0;
        bar_hessian = bar_hessian*0;
    end
    phi = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
    hessian_inv = (cost_hessian+bar_hessian)^-1;
    dot_pb_hat = -hessian_inv*phi;
    % cost function
    Acost = eye(link_num);
    bcost = th_ref';
    dAcost = zeros(link_num);
    dbcost = dot_th_ref';
    [cost,cost_nabla,cost_nabla_dot,cost_hessian] = quadcost(th,t,Acost,bcost,dAcost,dbcost);
    % barrier
    [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier(th,t,Ath,bth);
    % cost+bar
    if imag(bar)
        bar_nabla = bar_nabla*0;
        bar_nabla_dot = bar_nabla_dot*0;
        bar_hessian = bar_hessian*0;
    end
    phi = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
    hessian_inv = (cost_hessian+bar_hessian)^-1;
    dot_th_hat = -hessian_inv*phi;
    % th optimization
    dot_pbe_hat = dot_pe_ref'-dot_pb_hat;
    dth_nom(1,:) = pinv(jac)*dot_pbe_hat+(eye(link_num)-pinv(jac)*jac)*dot_th_hat;
    dot_th = dth_nom;
    % extended K-class function
    akcf = 1;
    ekcf = @(h) akcf*h;
    % cbf - quadratic program
    Aqp = [-Areg*jac,ekcf(Areg*(pe_ref'-fk')-breg)];
    bqp = -Areg*dot_pe_ref';
    for j=2:link_num
        Aqp = [Aqp;-Areg*nabla(th,j),ekcf(Areg*(pe_ref'-hinge(th,j)')-breg)];
        bqp = [bqp;-Areg*dot_pe_ref'];
    end
    Aqp = [Aqp;Ath,ekcf(Ath*th'-bth);zeros(1,link_num),-1];
    bqp = [bqp;zeros(size(Ath,1),1);0];
    [uqp(1,:),f,is_solved] = quadprog(eye(link_num+1),[-dth_nom';-1],Aqp,bqp);
    dot_th = uqp(1:link_num);
    k = uqp(end);
    % update'
    r = r+dt*dot_r;
    th = th+dt*dot_th;
    pe_ref = pe_ref+dt*dot_pe_ref;
    pb_ref = pe_ref-fkine(th_ref);
    % plot
    pb = pe_ref-fk;
    robot_desired.animate(th,[pb,0]);
    set(fpe,'xdata',pe_ref(1),'ydata',pe_ref(2));
    set(fpbr,'xdata',pb_ref(1),'ydata',pb_ref(2));
    xcirc = circle_(pe_ref,r);
    set(fcirc,'xdata',xcirc(:,1),'ydata',xcirc(:,2));
    if video_on
       frame(loop)= getframe(gcf); 
    end
    drawnow
end
% write video
if video_on 
    savevideo('robot_cbf',frame);
end

