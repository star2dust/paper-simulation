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

pe_ref = [-1.5,-1]; 
angle_ref = -pi*3/4; angle_safe = 0.9*pi;
th_ref = joint_ref_func(angle_ref,3/4);
pb_ref = pe_ref-fkine(th_ref);
Tc = [-sin(angle_ref-angle_safe/2),cos(angle_ref-angle_safe/2);
    sin(angle_ref+angle_safe/2),-cos(angle_ref+angle_safe/2)];

pb = [2,0]; pb_hat = pb;
th = robot_desired.ikine2d(pe_ref,[pb,pi])-[pi,zeros(1,link_num-1)];
dth = zeros(size(th));
th_ref = suitangle(th,th_ref);

V = [-1,4;6,-5];
kb = [V(:,1),ones(size(V,1),1)]^-1*V(:,2);
Abar = [-kb(1),1]/norm([-kb(1),1]);
bbar = kb(2)/norm([-kb(1),1]);
Ath = [tril(ones(link_num));-tril(ones(link_num))];
bth = kron([angle_ref+angle_safe/2;-angle_ref+angle_safe/2],ones(link_num,1));

figure
gca; hold on
robot_desired.plot(th,[pb,0]);
fpbr = plot3(pb_ref(1),pb_ref(2),2,'rs');
fpe = plot3(pe_ref(1),pe_ref(2),2,'co');
fpbh = plot3(pb_hat(1),pb_hat(2),2,'cs');
plot(V(:,1), V(:,2), 'ro-', 'LineWidth', 2);
% x = [pb_hat([1 1],1);pe_ref([1 1],1)];
% y = [pb_hat([1 1],2);pe_ref([1 1],2)];
% vec_in_min = [cos(angle_ref-angle_safe/2);sin(angle_ref-angle_safe/2)];
% vec_in_max = [cos(angle_ref+angle_safe/2);sin(angle_ref+angle_safe/2)];
% vec_out_min = [cos(angle_ref+pi-angle_safe/2);sin(angle_ref+pi-angle_safe/2)];
% vec_out_max = [cos(angle_ref+pi+angle_safe/2);sin(angle_ref+pi+angle_safe/2)];
% u = [vec_in_min(1);vec_in_max(1);
%     vec_out_min(1);vec_out_max(1)];
% v = [vec_in_min(2);vec_in_max(2);
%     vec_out_min(2);vec_out_max(2)];
% u = sum(link_list)/2*u;
% v = sum(link_list)/2*v;
% z = ones(size(x))*2;
% w = zeros(size(z));
% fcone = quiver(x,y,u,v,'zdata',z,'wdata',w,'linestyle','--','linewidth',1);
axis([-3 7 -5 5]);


% simulation
dt = 0.01;
T = 6;
loop = 0;
color_list = ['b','g','m','r','c'];
for t=0:dt:T
    loop = loop+1;
    % optmization
    dot_pe_ref = unit(diff(V))*rot2(pi/2)'/2;
    jacref = jacob(th_ref);
    fkref = fkine(th_ref);
    fkth = fkine(th);
    jacth = jacob(th);
%     Ab = blkdiag(Abar,Ath);
%     bb = [bbar;bth];
%     dAb = zeros(size(Ab));
%     dbb = zeros(size(bb));
%     Ac = Tc*[eye(2),jacref];
%     bc = Tc*(pe_ref'-fkref'+jacref*th_ref');
%     dAc = Tc*[eye(2),zeros(size(jacref))];
%     dbc = Tc*dot_pe_ref';
%     if t==0
%         x = convproj([pb_hat,th],[Ab;Ac],[bb;bc],[-ones(1,5);ones(1,5)]*10);
%         pb_hat = x(1:2); th = x(3:end);
%     end
%     [Ab;Ac]*[pb_hat,th]'-[bb;bc]
    [cost,cost_nabla,cost_nabla_dot,cost_hessian] = basecost(pb_hat,th,fkth,jacth,t,pe_ref,Tc,th_ref,dot_pe_ref);
    [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier(pb_hat,t,Abar,bbar);
%     [bar,bar_nabla,bar_nabla_dot,bar_hessian] = logbarrier([pb_hat,th],t,[Ab;Ac],[bb;bc],[dAb;dAc],[dbb;dbc]);
    if imag(bar)
        bar_nabla = bar_nabla*0;
        bar_nabla_dot = bar_nabla_dot*0;
        bar_hessian = bar_hessian*0;
    end
    cost+bar
    phi = cost_nabla+[bar_nabla;zeros(link_num,1)]+cost_nabla_dot+[bar_nabla_dot;zeros(link_num,1)];
    hessian_inv = (cost_hessian+blkdiag(bar_hessian,zeros(link_num)))^-1;
%     phi = cost_nabla+bar_nabla+cost_nabla_dot+bar_nabla_dot;
%     hessian_inv = (cost_hessian+bar_hessian)^-1;
    % algorithm
    uq = -hessian_inv*phi;
    % update
    pb_hat = pb_hat+dt*uq(1:2)';
    th = th+dt*uq(3:end)';
    pe_ref = pe_ref+dt*dot_pe_ref;
    pb_ref = pe_ref-fkine(th_ref);
    % plot
    pb = pe_ref-fkth;
    robot_desired.animate(th,[pb,0]);
    set(fpe,'xdata',pe_ref(1),'ydata',pe_ref(2));
    set(fpbh,'xdata',pb_hat(1),'ydata',pb_hat(2));
    set(fpbr,'xdata',pb_ref(1),'ydata',pb_ref(2));
%     x = [pb_hat([1 1],1);pe_ref([1 1],1)];
%     y = [pb_hat([1 1],2);pe_ref([1 1],2)];
%     vec_in_min = [cos(angle_ref-angle_safe/2);sin(angle_ref-angle_safe/2)];
%     vec_in_max = [cos(angle_ref+angle_safe/2);sin(angle_ref+angle_safe/2)];
%     vec_out_min = [cos(angle_ref+pi-angle_safe/2);sin(angle_ref+pi-angle_safe/2)];
%     vec_out_max = [cos(angle_ref+pi+angle_safe/2);sin(angle_ref+pi+angle_safe/2)];
%     u = [vec_in_min(1);vec_in_max(1);
%         vec_out_min(1);vec_out_max(1)];
%     v = [vec_in_min(2);vec_in_max(2);
%         vec_out_min(2);vec_out_max(2)];
%     u = sum(link_list)/2*u;
%     v = sum(link_list)/2*v;
%     set(fcone,'xdata',x,'ydata',y,'udata',u,'vdata',v)
    hold off
    drawnow
end


