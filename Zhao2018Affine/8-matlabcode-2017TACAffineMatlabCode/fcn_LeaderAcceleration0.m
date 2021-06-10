function output=fcn_LeaderAcceleration(u)
global leaderNum dim samplePos_all nodenum formationType
global rotateFlag tempp2 tempp3

u_all = reshape(u(1:dim*nodenum*3),dim,nodenum*3);
v_all=u_all(:,1:nodenum);
p_all=u_all(:,nodenum+1:2*nodenum);
control_all=u_all(:,2*nodenum+1:3*nodenum);
currentTime=u(end);
%output
a_leader_all=zeros(dim,leaderNum);

% speed up to speed=1
time_start_speedup=0;
time_span_speedup=5;
if currentTime>time_start_speedup && currentTime<time_start_speedup+time_span_speedup
    ax=fcn_sin(time_start_speedup,time_span_speedup,currentTime)/pi;
    a_leader_all(:,1)=[ax,0]';
    a_leader_all(:,2)=[ax,0]';
    a_leader_all(:,3)=[ax,0]';
end

% slow down to speed=0
time_start_slowdown=10;
time_span_slowdown=5;
if currentTime>time_start_slowdown && currentTime<time_start_slowdown+time_span_slowdown
    %ax=fcn_normalDistribution(time_start_slowdown,time_span_slowdown, 2, currentTime);
    ax=-fcn_sin(time_start_slowdown,time_span_slowdown,currentTime)/pi;
    a_leader_all(:,1)=[ax,0]';
    a_leader_all(:,2)=[ax,0]';
    a_leader_all(:,3)=[ax,0]';
end

% rotate
v3=v_all(:,3);
v2=v_all(:,2);
v1=v_all(:,1);
p3=p_all(:,3);
p2=p_all(:,2);
p1=p_all(:,1);
R90=[cos(pi/2),-sin(pi/2);
     sin(pi/2),cos(pi/2)];
time_start_rotate=20;
time_span_rotate=10;
if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate
    if rotateFlag==1
        tempp2=p2;
        tempp3=p3;
        rotateFlag=0;
    end
    % agent 2
    acce_mag=-fcn_SlowDownSpeedUp(time_start_rotate, time_span_rotate, currentTime, 4);
%     acce_vec=(p1+R90'*(tempp2-p1))-tempp2;
%     acce_vec=acce_vec/norm(acce_vec);
    acce_vec=[1 0]';
    a_leader_all(:,2)=acce_mag*acce_vec;
    % agent 3
    acce_mag=-fcn_SlowDownSpeedUp(time_start_rotate, time_span_rotate, currentTime, 4);
%     acce_vec=(p1+R90'*(tempp3-p1))-tempp3;
%     acce_vec=acce_vec/norm(acce_vec);
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

% slow down to speed=0
time_start_slowdown2=time_start_downward+time_span_downward+5;
time_span_slowdown2=5;
if currentTime>time_start_slowdown2 && currentTime<time_start_slowdown2+time_span_slowdown2
    %ax=fcn_normalDistribution(time_start_slowdown,time_span_slowdown, 2, currentTime);
    ay=-fcn_sin(time_start_slowdown2,time_span_slowdown2,currentTime)/pi;
    a_leader_all(:,1)=[0,-ay]';
    a_leader_all(:,2)=[0,-ay]';
    a_leader_all(:,3)=[0,-ay]';
end




% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate
%     % agent 3
%     aUnit=(p1-p3)/norm(p1-p3);
%     tht_dot=fcn_sin(time_start_rotate,time_span_rotate,currentTime);
%     speed_dot=fcn_sin(time_start_rotate,time_span_rotate,currentTime);
%     a_leader_all(:,3)=aUnit*tht_dot*norm(v3)+speed_dot*R90*aUnit;
%     % agent 2
%     aUnit=(p1-p2)/norm(p1-p2);
%     %tht_dot=fcn_sin(time_start_rotate,time_span_rotate,currentTime);
%     speed_dot=fcn_sin(time_start_rotate,time_span_rotate,currentTime);
%     a_leader_all(:,2)=aUnit*tht_dot*norm(v2)+speed_dot*R90*aUnit;
% end

% % scaling
% a_leader_all(2,2)=-fcn_TanhManeuver(20, 1, 10, currentTime);
% a_leader_all(2,3)=fcn_TanhManeuver(20, 1, 10, currentTime);

% % rotation
% v3=v_all(:,3);
% v2=v_all(:,2);
% v1=v_all(:,1);
% p3=p_all(:,3);
% p2=p_all(:,2);
% p1=p_all(:,1);
% R90=[cos(pi/2),-sin(pi/2);
%      sin(pi/2),cos(pi/2)];
% time_start_rotate=20;
% % agent 3
% time_span_rotate3=10;
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate3
%     if rotateCenterFlag==1
%         roateCenter(:,3)=p3-[0,5]';
%         rotateCenterFlag=0;
%         r3=norm(roateCenter(:,3)-p3);
%         roateCenter(:,2)=roateCenter(:,3);
%         r2=norm(roateCenter(:,2)-p2);
%         roateCenter(:,1)=roateCenter(:,3);
%         r1=norm(roateCenter(:,1)-p1);
%     end
%     tht_dot=pi/time_span_rotate3;
%     speed_dot=0;
%     a_leader_all(:,3)=R90'*v3/norm(v3)*tht_dot*norm(v3)+speed_dot*v3/norm(v3);
% end
% % agent 2
% time_span_rotate2=time_span_rotate3;
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate2
%     tht_dot=pi/time_span_rotate2;
%     if currentTime>=time_start_rotate && currentTime<=time_start_rotate+1
%         speed_dot=fcn_sin(time_start_rotate,1,currentTime)/2.6;
%     else 
%         speed_dot=0;
%     end
%     a_leader_all(:,2)=R90'*v2/norm(v2)*tht_dot*norm(v2)+speed_dot*v2/norm(v2);
% end
% if currentTime>=time_start_rotate+time_span_rotate2 && currentTime<=time_start_rotate+time_span_rotate2+1
%         speed_dot=-fcn_sin(time_start_rotate,1,currentTime)/2.6;
%         tht_dot=0;
%         a_leader_all(:,2)=R90'*v2/norm(v2)*tht_dot*norm(v2)+speed_dot*v2/norm(v2);
% end
% % agent 1
% time_span_rotate1=time_span_rotate3;
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate1
%     tht_dot=pi/time_span_rotate3;%fcn_sin(time_start_rotate,time_span_rotate1,currentTime);
%     if currentTime>=time_start_rotate && currentTime<=time_start_rotate+1
%         speed_dot=fcn_sin(time_start_rotate,1,currentTime)/3;
% %         tht_dot=
%     else 
%         speed_dot=0;
%     end
%     aUnit=(roateCenter(:,1)-p1)/norm(roateCenter(:,1)-p1);
%     a_leader_all(:,1)=aUnit*tht_dot*norm(v1)+speed_dot*v1/norm(v1);
% end
% if currentTime>=time_start_rotate+time_span_rotate2 && currentTime<=time_start_rotate+time_span_rotate2+1
%         speed_dot=-fcn_sin(time_start_rotate,1,currentTime)/2.6;
%         tht_dot=0;
%         a_leader_all(:,2)=R90'*v2/norm(v2)*tht_dot*norm(v2)+speed_dot*v2/norm(v2);
% end


% % agent 1: distance-based formation control law
% time_span_rotate1=time_span_rotate3;
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate1
%     delta12=norm(p1-p2)-2*sqrt(2);
%     delta13=norm(p1-p3)-2*sqrt(2);
%     a_leader_all(:,1)=5*(delta12*(p2-p1)+delta13*(p3-p1)) + 0.5*(delta12*(v2-v1)+delta13*(v3-v1));
% end

% % agent 2
% time_span_rotate2=26;
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate2
%     v2=v_all(:,2);
%     p2=p_all(:,2);
%     R90=[cos(pi/2),-sin(pi/2);
%              sin(pi/2),cos(pi/2)];
%     tht_dot=fcn_sin(time_start_rotate,time_span_rotate2,currentTime);
%     speed_dot=0;
%     a_leader_all(:,2)=R90'*v2/norm(v2)*tht_dot*norm(v2)+speed_dot*v2/norm(v2);
% end
% % agent 1
% time_span_rotate1=18;
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate1
%     v1=v_all(:,1);
%     p1=p_all(:,1);
%     R90=[cos(pi/2),-sin(pi/2);
%              sin(pi/2),cos(pi/2)];
%     tht_dot=fcn_sin(time_start_rotate,time_span_rotate1,currentTime);
%     speed_dot=0;
%     a_leader_all(:,1)=R90'*v1/norm(v1)*tht_dot*norm(v1)+speed_dot*v1/norm(v1);
% end
% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate
%     v3=v_all(:,3);
%     v2=v_all(:,2);
%     v1=v_all(:,1);
%     p3=p_all(:,3);
%     p2=p_all(:,2);
%     p1=p_all(:,1);
%     R90=[cos(pi/2),-sin(pi/2);
%          sin(pi/2),cos(pi/2)];
%     % agent 3
%     tht_dot=-fcn_normalDistribution(time_start_rotate,time_span_rotate, 3.15, currentTime);
%     speed_dot=0;
%     a_leader_all(:,3)=R90'*v3/norm(v3)*tht_dot*norm(v3)+speed_dot*v3/norm(v3);
%     % agent 2
%     tht_dot=-fcn_normalDistribution(time_start_rotate,time_span_rotate*1.5, 3.15, currentTime);
%     speed_dot=0;
%     a_leader_all(:,2)=R90'*v2/norm(v2)*tht_dot*norm(v2)+speed_dot*v2/norm(v2);
%     % agent 1
%     tht_dot=-fcn_normalDistribution(time_start_rotate,time_span_rotate*, 2, currentTime);
%     speed_dot=0;%fcn_normalDistribution(time_start_rotate,time_span_rotate, 1, currentTime);
%     a_leader_all(:,1)=R90'*v1/norm(v1)*tht_dot*norm(v1)+speed_dot*v1/norm(v1);
% end


% if currentTime>=time_start_rotate && currentTime<=time_start_rotate+time_span_rotate
%     offset=4;
%     v3=v_all(:,3);
%     v2=v_all(:,2);
%     v1=v_all(:,1);
%     p3=p_all(:,3);
%     p2=p_all(:,2);
%     p1=p_all(:,1);
%     R90=[cos(pi/2),-sin(pi/2);
%          sin(pi/2),cos(pi/2)];
%     threshhold=0.005;
%     if rotateCenterFlag==1
%         roateCenter(:,3)=p3-[0,offset]';
%         rotateCenterFlag=0;
%         r3=norm(roateCenter(:,3)-p3);
%         rotateTime(3)=pi*r3/norm(v3);
%         roateCenter(:,2)=roateCenter(:,3);
%         r2=norm(roateCenter(:,2)-p2);
%         rotateTime(2)=pi*r2/norm(v2);
% %         roateCenter(:,1)=roateCenter(:,3)+(p1-p3);
%         roateCenter(:,1)=roateCenter(:,2)+[p1(1)-p2(1),0]';
%         r1=norm(roateCenter(:,1)-p1);
%         rotateTime(1)=pi*r1/norm(v1);
%     end
%     % agent 3
%     tht=atan2(v3(2),v3(1));
%     if ~(tht>pi-threshhold || tht<-pi+threshhold)
%         r3=norm(roateCenter(:,3)-p3);
%         aUnit=(roateCenter(:,3)-p3)/norm(roateCenter(:,3)-p3);
%         a_leader_all(:,3)=aUnit*norm(v3)^2/r3;%+speed_dot*v3/norm(v3);
%     end
%     % agent 2
%     tht=atan2(v2(2),v2(1));
%     if ~(tht>pi-threshhold || tht<-pi+threshhold)
%         r2=norm(roateCenter(:,2)-p2);
%         aUnit=(roateCenter(:,2)-p2)/norm(roateCenter(:,2)-p2);
%         speed_dot=-fcn_SlowDownSpeedUp(time_start_rotate, 5, currentTime, 8);
%         a_leader_all(:,2)=aUnit*norm(v2)^2/r2;%+speed_dot*v2/norm(v2);
%     end
%     % agent 1
%     tht=atan2(v1(2),v1(1));
%     if ~(tht>pi-threshhold || tht<-pi+threshhold)
%         r1=norm(roateCenter(:,1)-p1);
%         aUnit=(roateCenter(:,1)-p1)/norm(roateCenter(:,1)-p1);
%         speed_dot=-fcn_SlowDownSpeedUp(time_start_rotate, 5, currentTime, 8);
%         a_leader_all(:,1)=aUnit*norm(v1)^2/r1;%+speed_dot*v1/norm(v1);
%     end
% end

% % slow down and speed up to align the agents
% time_start_align=20;
% time_span=5;
% if currentTime>time_start_align && currentTime<time_start_align+time_span
% %     % agent 3: back 4 meters
% %     distance_change=4;
% %     a_leader_all(2,3)=-fcn_SlowDownSpeedUp(time_start_align, time_span, currentTime, distance_change);
% %     % agent 2: foward 2 meters
% %     distance_change=2.3;
% %     a_leader_all(2,2)=fcn_SlowDownSpeedUp(time_start_align, time_span, currentTime, distance_change);
%     % agent 1 slow down
%     distance_change=2;
%     a_leader_all(1,1)=fcn_SlowDownSpeedUp(time_start_align, time_span, currentTime, distance_change);
% end



% output: replace the leaders' velocity with the one prescribed above
control_all(:,1:leaderNum) = a_leader_all;
output=reshape(control_all, dim*nodenum, 1);



