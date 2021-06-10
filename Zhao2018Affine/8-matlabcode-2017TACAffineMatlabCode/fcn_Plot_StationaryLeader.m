function fcn_Plot_StationaryLeader(p_all_time, v_all_time, a_all_time, error_all, aDiff_all)
close all

global nodenum edgenum neighborMat dim simTime leaderNum kp kv
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot a subset of the data, because too much data
delta=50;
time_all=v_all_time.time(1:delta:end);
p_all=p_all_time.signals.values(1:delta:end,:);
v_all=v_all_time.signals.values(1:delta:end,:);
a_all=a_all_time.signals.values(1:delta:end,:);
error_all=error_all.signals.values(1:delta:end,:);
aDiff_all=aDiff_all.signals.values(1:delta:end,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
linewidth=0.5;
dotsize=7;
fontsize=7;%15;
formationColor=0*[0 0 1];
% edgeColorList=zeros(nodenum,3);
faceColorList=zeros(nodenum,3);
for i=1:nodenum
    if i<=leaderNum
        faceColorList(i,:)=[1,0,0];
    else
        faceColorList(i,:)=[0,0,1];
    end
end
for i=1:nodenum
    if i<=leaderNum
        markerList(i)={'o'};
    else
        markerList(i)={'o'};
    end
end
for i=1:nodenum
    if i<=leaderNum
        linestyleList(i)={'-'};
    else
        linestyleList(i)={'--'};
    end
end
%% plot trajectory
figure; 
subplot(4,1,[1,2,3]); % to make the axis short
hold on; box on; 
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'x (m)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', fontsize);
%set(get(gca, 'zlabel'), 'String', 'z (m)', 'fontSize', fontsize);
axis equal
set(gca, 'fontsize', fontsize);
%plot trajectory
for i=1:nodenum
    if dim==2
        xi_all=p_all(:,2*i-1);
        yi_all=p_all(:,2*i);
        plot(xi_all, yi_all, ':', 'linewidth', linewidth, 'color', faceColorList(i,:));
    else
        xi_all=p_all(:,3*i-2);
        yi_all=p_all(:,3*i-1);
        zi_all=p_all(:,3*i);
        plot3(xi_all, yi_all, zi_all, ':', 'linewidth', linewidth, 'color', faceColorList(i,:));
    end
end
% plot obstacle
leftx=18;
width=5;
h=rectangle('Position', [leftx, 1.8, width, 15]);
set(h, 'faceColor', 0.4*ones(1,3))
h=rectangle('Position', [leftx, -16.5, width, 14.5]);
set(h, 'faceColor', 0.4*ones(1,3))
h=rectangle('Position', [leftx, -25, width, 6.5]);
set(h, 'faceColor', 0.4*ones(1,3))
text(20.5, -9, 'Obstacle','color', 'w', 'FontSize', 8, 'horizontalAlignment', 'center', 'FontWeight', 'normal', 'Rotation', 0, 'fontname', 'Times');

axis tight
xlim=get(gca,'xlim');
set(gca,'xlim', xlim+[-5,5]);
ylim=get(gca,'ylim');
set(gca,'ylim', ylim+[-4,3]);
% title
%set(get(gca, 'title'), 'String', 'Trajectory', 'fontsize', fontsize);

%% plot intermediate formations
dataNum=size(p_all,1);
hMarkerAll=zeros(nodenum,1);
index=[1,floor(dataNum/10*1.4),floor(dataNum/10*2.2),floor(dataNum/10*3.2),floor(dataNum/10*4.47),...
    floor(dataNum/10*5.3),floor(dataNum/10*6.45),floor(dataNum/10*7.2),floor(dataNum/10*8),floor(dataNum/10*9),floor(dataNum/10*10)]; % for 2D scale big example
% index=[1,floor(dataNum/10*10)];
for k=1:size(index,2)
    idx=index(k);
    for i=1:nodenum
        for j=1:nodenum
            if neighborMat(i,j)==1
                if dim==2
                    pi=p_all(idx,2*i-1:2*i)';
                    pj=p_all(idx,2*j-1:2*j)';
                    line([pi(1),pj(1)], [pi(2),pj(2)], 'linewidth', 0.5, 'color', formationColor);
                    %fcn_plotArrow((pi+pj)/2, (pj-pi)/norm(pj-pi), initialColor, linewidth)
                else
                    pi=p_all(idx,3*i-2:3*i)';
                    pj=p_all(idx,3*j-2:3*j)';
                    line([pi(1),pj(1)], [pi(2),pj(2)], [pi(3),pj(3)], 'linewidth', 0.5, 'color', formationColor);
                    %fcn_plotArrow((pi+pj)/2, (pj-pi)/norm(pj-pi),
                    %initialColor, linewidth)
                end
            end
        end
    end    
    for i=1:nodenum
        if dim==2
            xi=p_all(idx,2*i-1);
            yi=p_all(idx,2*i);
            hMarkerAll(i)=plot(xi, yi, char(markerList(i)), 'MarkerEdgeColor', 'k', 'MarkerFaceColor', faceColorList(i,:), 'markersize', dotsize, 'linewidth', 0.5);
            text(xi, yi, num2str(i),'color', 'w', 'FontSize', 5, 'horizontalAlignment', 'center', 'FontWeight', 'bold');
        else
            xi=p_all(idx,3*i-2);
            yi=p_all(idx,3*i-1);
            zi=p_all(idx,3*i);
            hMarkerAll(i)=plot3(xi, yi, zi, char(markerList(i)), 'MarkerEdgeColor', 'k', 'MarkerFaceColor', faceColorList(i,:), 'markersize', dotsize, 'linewidth', 1);
        end
    end
end
hLegendTraj=legend([hMarkerAll(1),hMarkerAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'West');
%set(hLegendTraj, 'pos', [0.14, 0.72, 0.2375, 0.1333]);
% axis tight
% margin=1;
% xlim=get(gca,'xlim');
% set(gca,'xlim', xlim+[-margin,margin]);
% ylim=get(gca,'ylim');
% set(gca,'ylim', ylim+[-margin,margin]);

% plot time for corresponding formation
text(0, 5, strcat('t=',num2str(time_all(index(1)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(10, 5, strcat('t=',num2str(time_all(index(2)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(33, 5, strcat('t=',num2str(time_all(index(4)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(45, 5, strcat('t=',num2str(time_all(index(5)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(50, -7, strcat('t=',num2str(time_all(index(6)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(50, -17, strcat('t=',num2str(time_all(index(7)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(38, -22, strcat('t=',num2str(time_all(index(8)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(28, -22, strcat('t=',num2str(time_all(index(9)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(14, -22, strcat('t=',num2str(time_all(index(10)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');
text(3, -22, strcat('t=',num2str(time_all(index(11)),'%.1f'),'s'),'color', 'k', 'FontSize', fontsize+1, 'horizontalAlignment', 'center', 'FontName', 'times');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot error
figure
subplot(5,1,1) % to make the axis short
hold on; box on
% set the format of the axis
set(gca, 'fontSize', fontsize)
% set(get(gca, 'xlabel'), 'String', 'Time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'Tracking error', 'fontSize', fontsize);
set(gca, 'xlim', [0,simTime])
% set(gca, 'ylim', [0,10])
plot(time_all, error_all, 'color', 'm', 'linewidth', 1)
% set(get(gca, 'title'), 'String', strcat('Tracking error: kp=',num2str(kp),', kv=',num2str(kv)), 'fontsize', fontsize);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot velocity
% figure
hLineAll=zeros(nodenum,1);
if dim==2
    % x velocity
    subplot(5,1,2)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, v_all(:,2*i-1), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'x-velocity (m/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    set(gca, 'ylim', [-3,3])
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
    % y velocity
    subplot(5,1,3)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, v_all(:,2*i), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'y-velocity (m/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    set(gca, 'ylim', [-3,3])
%     set(get(gca, 'xlabel'), 'String', 'Time (s)', 'fontSize', fontsize);
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
elseif dim==3 % plot vx and vy
    % x velocity
    subplot(3,1,1)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, v_all(:,3*i-2), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'x-velocity (m/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
    % title
    set(get(gca, 'title'), 'String', 'Velocity', 'fontsize', fontsize);
    % y velocity
    subplot(3,1,2)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, v_all(:,3*i-1), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'y-velocity (m/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    hLegend=legend([hLineAll(1),hLineAll(3)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
    % z velocity
    subplot(3,1,3)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, v_all(:,3*i), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'xlabel'), 'String', 'Time (sec)', 'fontSize', fontsize);
    set(get(gca, 'ylabel'), 'String', 'z-velocity (m/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    hLegend=legend([hLineAll(1),hLineAll(3)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot acceleration
% figure
hLineAll=zeros(nodenum,1);
if dim==2
    % x velocity
    subplot(5,1,4)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, a_all(:,2*i-1), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'x-acceleration (m^2/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
    % title
%     set(get(gca, 'title'), 'String', 'Acceleration', 'fontsize', fontsize);
    % y velocity
    subplot(5,1,5)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, a_all(:,2*i), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'y-acceleration (m^2/s)', 'fontSize', fontsize);
    set(get(gca, 'xlabel'), 'String', 'Time (s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot acceleration error
figure
hLineAll=zeros(nodenum,1);
if dim==2
    % x velocity
    subplot(5,1,1)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, aDiff_all(:,2*i-1), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'x-aDiff (m^2/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    % y velocity
    subplot(5,1,2)
    hold on; box on;
    for i=1:nodenum
        hLineAll(i)=plot(time_all, aDiff_all(:,2*i), 'color', faceColorList(i,:), 'linestyle', char(linestyleList(i)), 'linewidth', 1);
    end
    set(gca, 'fontSize', fontsize)
    set(get(gca, 'ylabel'), 'String', 'y-aDiff (m^2/s)', 'fontSize', fontsize);
    set(gca, 'xlim', [0,simTime])
    set(get(gca, 'xlabel'), 'String', 'Time (s)', 'fontSize', fontsize);
    hLegend=legend([hLineAll(1),hLineAll(leaderNum+1)], 'Leader', 'Follower', 'location', 'southwest');
    set(hLegend, 'fontsize', fontsize);
end


