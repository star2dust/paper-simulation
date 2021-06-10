clc; clear; close all
global nodenum neighborMat stressMat dim simTime leaderNum ifLeaderFollower kp kv samplePos_all formationType simTime
global rotateFlag tempp1 tempp2
%
ifLeaderFollower=1;
leaderNum=3; % the first leaderNum agents are leaders
dim=2;
% configuration matrix
P=2*[2 0;
   1 1;
   1 -1;
   0 1;
   0 -1;
   -1 1;
   -1 -1];
nodenum=size(P,1); % number of nodes
% adjacent matrix
neighborMat=zeros(nodenum,nodenum);
neighborMat(1,2)=1;neighborMat(1,3)=1;neighborMat(1,4)=1;neighborMat(1,5)=1;
neighborMat(2,4)=1;
neighborMat(3,5)=1;neighborMat(3,6)=1;
neighborMat(4,5)=1;neighborMat(4,6)=1;neighborMat(2,7)=1;
neighborMat(5,7)=1;
neighborMat(6,7)=1;
m=sum(sum(neighborMat)); % number of edges
neighborMat=neighborMat+neighborMat';
% stress matrix: the returned is -Omega, hence the ijth entry is omega_ij!!
stressMat=fcn_StressMatrix(dim,nodenum,m,P,neighborMat); 

kv=2;kp=0.5; %good
samplePos_all=P';
p=reshape(P',dim*nodenum,1);
% initial states
x_init=p+[0 0 ...
   0 0 ...
   0 0 ...
   1 1 ...
   0 1 ...
   -1 1 ...
   -2 -1]';
for k=1:leaderNum
	x_init(dim*(k-1)+1:dim*k)=samplePos_all(:,k); % initial leader position
end
v_leader_init=zeros(dim*leaderNum,1);
v_init=zeros(dim*nodenum,1);
v_init(1:dim*leaderNum,1)=v_leader_init; % initial leader velocity

rotateFlag=1;
tempp1=zeros(dim,1);
tempp2=zeros(dim,1);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% simulink
simTime=125
stepsize=0.005 
% comment: reduce the stepsize to get less tracking error; 
% that is because there is acceleration feedback, which is delayed by the time of stepsize.
% the value used in the paper is 0.001
option=simset('fixedstep',stepsize);
sim('affineManeuver.mdl', [0, simTime], option)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot result
fcn_Plot_StationaryLeader(p_all_time, v_all_time, a_all, error_all, aDiff_all)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% animation
fcn_Animation_2DObstacle(p_all_time, error_all)


