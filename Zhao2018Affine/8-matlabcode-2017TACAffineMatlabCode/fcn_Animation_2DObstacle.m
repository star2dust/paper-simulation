function fcn_Animation_2DObstacle(p_all_time, error_all)
global nodenum edgenum neighborMat dim simTime leaderNum ki kp
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% animation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
% mov = avifile('video1.avi');
figure;
set(gcf, 'unit', 'norm', 'pos', [0.1,0.1,0.72,0.8])
hAxisTraj=subplot(4,1,1:3);
hold on; box on; axis equal
xlim([-10,55])
ylim([-25,6])
%%%%%%%%%%%%%%%%%%%%%
% plot a subset of the data, because too much data
delta=50;
time_all=p_all_time.time(1:delta:end);
p_all=p_all_time.signals.values(1:delta:end,:);
error_all=error_all.signals.values(1:delta:end,:);
%%%%%%%%%%%%%%%%%%%%%%%
linewidth=0.5;
fontsize=7;
dotsize=9;
formationColor=0*[0 0 1];
% edgeColorList=zeros(nodenum,3);
for i=1:nodenum
    if i<=leaderNum
        edgeColorList(i,:)=[1,0,0];
    else
        edgeColorList(i,:)=[0,0,1];
    end
end
faceColorList=edgeColorList;
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'x (meter)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'y (meter)', 'fontSize', fontsize);
% set(get(gca, 'title'), 'String', strcat('kp=',num2str(kp),', ki=',num2str(ki)), 'fontsize', fontsize);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot initial formation and leave it there
for i=1:nodenum
    for j=1:nodenum
        if neighborMat(i,j)==1
            pi=p_all(1,2*i-1:2*i)';
            pj=p_all(1,2*j-1:2*j)';
            line([pi(1),pj(1)], [pi(2),pj(2)], 'linewidth', 1, 'color', formationColor);
        end
    end
    xi=p_all(1,2*i-1);
    yi=p_all(1,2*i);
    plot(xi, yi, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', faceColorList(i,:), 'markersize', dotsize-2, 'linewidth', 1)
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%% ======================================
% fist time: create objects for the initial formation
% >>>objects of the lines
hLine=zeros(nodenum,nodenum);
for i=1:nodenum
    for j=i+1:nodenum
        if neighborMat(i,j)==1
            pi=p_all(1,2*i-1:2*i)';
            pj=p_all(1,2*j-1:2*j)';
            hLine(i,j)=line([pi(1),pj(1)], [pi(2),pj(2)], 'linewidth', linewidth, 'color', formationColor);
        end
    end
end
% >>>objects of the dots
hMarker=zeros(1,nodenum);
hText=zeros(1,nodenum);
for i=1:nodenum
    xi=p_all(1,2*i-1);
    yi=p_all(1,2*i);
    hMarker(i) = plot(xi, yi, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', faceColorList(i,:), 'markersize', dotsize, 'linewidth', 1);
    hText(i)=text(xi, yi, num2str(i),'color', 'w', 'FontSize', fontsize, 'horizontalAlignment', 'center', 'FontWeight', 'bold');
end
hLegend=legend([hMarker(1),hMarker(leaderNum+1)], 'Leader', 'Follower', 'location', 'west');
set(hLegend, 'fontsize', fontsize);
% >>>objects of the trajectories
hTraj=zeros(1,nodenum);
for i=1:nodenum
    xi_all=p_all(1,2*i-1);
    yi_all=p_all(1,2*i);
    hTraj(i) = plot(xi_all, yi_all, ':', 'linewidth', 1, 'color', edgeColorList(i,:));
end
%% =========================================
% first time: tracking error
hAxisError=subplot(4,1,4);
hold on; box on
% set the format of the axis
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'Time (second)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'Tracking error', 'fontSize', fontsize);
set(gca, 'xlim',[0 125])
set(gca, 'ylim', [0 4])
hError=plot(time_all(1), error_all(1),  'm', 'linewidth', 2);
%% ===============================
% every time: update the position of each object
for k=1:4:size(p_all, 1)
    for i=1:nodenum
        %
        xi_all=p_all(1:k,2*i-1);
        yi_all=p_all(1:k,2*i);
        set(hTraj(i), 'xdata', xi_all, 'ydata', yi_all);
        %
        xi=p_all(k,2*i-1);
        yi=p_all(k,2*i);
        set(hMarker(i), 'xdata', xi, 'ydata', yi);
        set(hText(i), 'Position', [xi,yi]);
        %
        for j=i+1:nodenum
            if neighborMat(i,j)==1
                pi=p_all(k,2*i-1:2*i)';
                pj=p_all(k,2*j-1:2*j)';
                set( hLine(i,j), 'xdata', [pi(1),pj(1)], 'ydata', [pi(2),pj(2)]);
            end
        end
        % update error
        set(hError, 'xdata', time_all(1:k), 'ydata', error_all(1:k));
%         errorYLim=get(hAxisError, 'ylim');
%         if error_all.signals.values(k)<0.4 && errorYLim(2)>0.4
%             set(hAxisError, 'ylim', errorYLim-[0, 0.08])
%         end
    end
    pause(0.1)
%     frame=getframe(gcf);
%     mov = addframe(mov,frame); 
end

% mov = close(mov);
