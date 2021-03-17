close all
clear

% example 1
% topology
L_l = [-1 1 0 0 0;
    0 -1 1 0 0;
    0 0 -1 1 0;
    0 0 0 -1 1];
L_s = [-1 1 1 0 -1 0 0;
    -1 0 1 1 0 -1 0;
    0 -1 0 0 2 0 -1];
L_d = [zeros(1,size(L_l,2)),zeros(1,size(L_s,1));
    L_l,zeros(size(L_l,1),size(L_s,1));
    zeros(size(L_s,1),1),L_s];
L_l1 = L_l(:,2:end);
L_l2 = L_l(:,1);
L_s1 = L_s(:,end-size(L_s,1)+1:end);
L_s2 = L_s(:,1:end-size(L_s,1));
% initial position
q = rand(size(L_d,1)-1,2);
q0 = rand(1,2);
-(L_l1^-1*L_l2)*q0
