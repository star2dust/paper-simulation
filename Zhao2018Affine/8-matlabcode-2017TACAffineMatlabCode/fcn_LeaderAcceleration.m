function output=fcn_LeaderAcceleration(u)
global leaderNum dim samplePos_all nodenum formationType simTime
global rotateFlag tempp2 tempp3

u_all = reshape(u(1:dim*nodenum*3),dim,nodenum*3);
v_all=u_all(:,1:nodenum);
p_all=u_all(:,nodenum+1:2*nodenum);
control_all=u_all(:,2*nodenum+1:3*nodenum);
currentTime=u(end);
if currentTime-floor(currentTime)==0
    disp(strcat('Completed %', num2str(currentTime/simTime*100,'%.1f')))
end
%output
a_leader_all=zeros(dim,leaderNum);

% speed up to speed=1
time_start_speedup=0;
time_span_speedup=15;
if currentTime>time_start_speedup && currentTime<time_start_speedup+time_span_speedup
    ax=fcn_sin(time_start_speedup,time_span_speedup,currentTime)/pi;
    a_leader_all(:,1)=[ax,0]';
    a_leader_all(:,2)=[ax,0]';
    a_leader_all(:,3)=[ax,0]';
end

% scale down to pass through obstacles
time_start_scaledown=time_start_speedup+time_span_speedup;
time_span_scaledown=10;
if currentTime>=time_start_scaledown && currentTime<=time_start_scaledown+time_span_scaledown
    % agent 3
    a_leader_all(2,3)=-fcn_SlowDownSpeedUp(time_start_scaledown, time_span_scaledown, currentTime, 1);
    % agent 3
    a_leader_all(2,2)=fcn_SlowDownSpeedUp(time_start_scaledown, time_span_scaledown, currentTime, 1);
end
% scale up after passing through obstacles
time_start_scaleup=time_start_scaledown+time_span_scaledown+5;
time_span_scaleup=10;
if currentTime>=time_start_scaleup && currentTime<=time_start_scaleup+time_span_scaleup
    % agent 3
    a_leader_all(2,3)=fcn_SlowDownSpeedUp(time_start_scaleup, time_span_scaleup, currentTime, 1);
    % agent 3
    a_leader_all(2,2)=-fcn_SlowDownSpeedUp(time_start_scaleup, time_span_scaleup, currentTime, 1);
end

% slow down to speed=0
time_start_slowdown=time_start_scaleup+time_span_scaleup+5;
time_span_slowdown=5;
if currentTime>time_start_slowdown && currentTime<time_start_slowdown+time_span_slowdown
    %ax=fcn_normalDistribution(time_start_slowdown,time_span_slowdown, 2, currentTime);
    ax=-fcn_sin(time_start_slowdown,time_span_slowdown,currentTime)/pi;
    a_leader_all(:,1)=[ax,0]';
    a_leader_all(:,2)=[ax,0]';
    a_leader_all(:,3)=[ax,0]';
end

% rotate
time_start_rotate=time_start_slowdown+time_span_slowdown;
time_span_rotate=10;
if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate
    % agent 2
    acce_mag=-fcn_SlowDownSpeedUp(time_start_rotate, time_span_rotate, currentTime, 4);
    acce_vec=[1 0]';
    a_leader_all(:,2)=acce_mag*acce_vec;
    % agent 3
    acce_mag=-fcn_SlowDownSpeedUp(time_start_rotate, time_span_rotate, currentTime, 4);
    acce_vec=[0 1]';
    a_leader_all(:,3)=acce_mag*acce_vec;
end

% move downward
time_start_downward=time_start_rotate;
time_span_downward=time_span_rotate;
if currentTime>=time_start_downward && currentTime<=time_start_downward+time_span_downward
    % agent 1
    ay=fcn_sin(time_start_downward,time_span_downward,currentTime)/pi;
    a_leader_all(:,1)=[0,-ay]';
    % agent 2
    a_leader_all(:,2)=a_leader_all(:,2)+[0,-ay]';
    % agent 3
    a_leader_all(:,3)=a_leader_all(:,3)+[0,-ay]';
end

% % one leader move to avoid obstacle
% time_start_agent2rightward=time_start_downward+time_span_rotate+5;
% time_span_agent2rightward=10;
% if currentTime>=time_start_agent2rightward && currentTime<=time_start_agent2rightward+time_span_agent2rightward
%     % agent 2
%     a_leader_all(1,2)=-fcn_SlowDownSpeedUp(time_start_agent2rightward, time_span_agent2rightward, currentTime, 4);
%     a_leader_all(1,3)=fcn_SlowDownSpeedUp(time_start_agent2rightward, time_span_agent2rightward, currentTime, 4);
% end
% %
% time_start_agent2leftward=time_start_agent2rightward+time_span_agent2rightward+5;
% time_span_agent2leftward=10;
% if currentTime>=time_start_agent2leftward && currentTime<=time_start_agent2leftward+time_span_agent2leftward
%     % agent 2
%     a_leader_all(1,2)=fcn_SlowDownSpeedUp(time_start_agent2leftward, time_span_agent2leftward, currentTime, 4);
%     a_leader_all(1,3)=-fcn_SlowDownSpeedUp(time_start_agent2leftward, time_span_agent2leftward, currentTime, 4);
% end

% slow down to speed=0
time_start_slowdown2=time_start_rotate+time_span_downward+10;
time_span_slowdown2=5;
if currentTime>time_start_slowdown2 && currentTime<time_start_slowdown2+time_span_slowdown2
    %ax=fcn_normalDistribution(time_start_slowdown,time_span_slowdown, 2, currentTime);
    ay=-fcn_sin(time_start_slowdown2,time_span_slowdown2,currentTime)/pi;
    a_leader_all(:,1)=[0,-ay]';
    a_leader_all(:,2)=[0,-ay]';
    a_leader_all(:,3)=[0,-ay]';
end

% rotate
time_start_rotate2=time_start_slowdown2+time_span_slowdown2;
time_span_rotate2=10;
if currentTime>=time_start_rotate2 && currentTime<=time_start_rotate2+time_span_rotate2
    % agent 2
    acce_mag=-fcn_SlowDownSpeedUp(time_start_rotate2, time_span_rotate2, currentTime, 4);
    acce_vec=[0 -1]';
    a_leader_all(:,2)=acce_mag*acce_vec;
    % agent 3
    acce_mag=-fcn_SlowDownSpeedUp(time_start_rotate2, time_span_rotate2, currentTime, 4);
    acce_vec=[1 0]';
    a_leader_all(:,3)=acce_mag*acce_vec;
end

% speed up to move leftward
time_start_leftward=time_start_rotate2;
time_span_leftward=time_span_rotate2;
if currentTime>=time_start_leftward && currentTime<=time_start_leftward+time_span_leftward
    % agent 1
    ax=-fcn_sin(time_start_leftward,time_span_leftward,currentTime)/pi;
    a_leader_all(:,1)=[ax,0]';
    % agent 2
    a_leader_all(:,2)=a_leader_all(:,2)+[ax,0]';
    % agent 3
    a_leader_all(:,3)=a_leader_all(:,3)+[ax,0]';
end

% affine transformation: agent 3 move forward+scale down in the vertical direction
time_start_affineVertical=time_start_leftward+time_span_leftward+5;
time_span_affineVertical=20;
if currentTime>=time_start_affineVertical && currentTime<=time_start_affineVertical+time_span_affineVertical
    % agent 3
    a_leader_all(1,3)=-fcn_SlowDownSpeedUp(time_start_affineVertical, time_span_affineVertical, currentTime, 1.5);
    %
    a_leader_all(2,3)=fcn_SlowDownSpeedUp(time_start_affineVertical, time_span_affineVertical, currentTime, 2);
    % agent 3
    a_leader_all(2,2)=-fcn_SlowDownSpeedUp(time_start_affineVertical, time_span_affineVertical, currentTime, 2);
end

% affine transformation: back to normal formation
time_start_backtonormal=time_start_affineVertical+time_span_affineVertical+10;
time_span_backtonormal=5;
if currentTime>=time_start_backtonormal && currentTime<=time_start_backtonormal+time_span_backtonormal
    % x-direction: agent 3
    a_leader_all(1,3)=fcn_SlowDownSpeedUp(time_start_backtonormal, time_span_backtonormal, currentTime, 1.5);
    % y-direction
    % agent 3
    a_leader_all(2,3)=-fcn_SlowDownSpeedUp(time_start_backtonormal, time_span_backtonormal, currentTime, 2);
    % agent 2
    a_leader_all(2,2)=fcn_SlowDownSpeedUp(time_start_backtonormal, time_span_backtonormal, currentTime, 2);
end

% output: replace the leaders' velocity with the one prescribed above
control_all(:,1:leaderNum) = a_leader_all;
output=reshape(control_all, dim*nodenum, 1);



