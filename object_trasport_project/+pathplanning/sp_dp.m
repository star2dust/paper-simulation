function [dst dist parent] = sp_dp(graph, v, dist,parent,Loc,Ind,deep)
% recursive function to solve dynamic programming for shortest path.
import pathplanning.*
deep=deep+1;
% find if coming node inside the already visited list
I=find(dist(:,1)==v);
if ~isempty(I)
    % if it is tru then return its cost
    dst= dist(I,2);
else   
    % if it is not true than add this node to the list
    dist=[dist;v inf];
    parent=[parent;v nan];
    % find all connected nodes to the current node
    u=find(graph(:,v)==1);
    I=size(dist,1);
    for i=1:length(u)    
        % find minimum distances way from nex connected node
        [dst2 dist parent] =sp_dp(graph,u(i),dist,parent,Loc,Ind,deep); 
        % calculate begin to cuurent plus next to end cost
        new_distance=costcal(Loc(Ind(u(i)),:),Loc(Ind(v),:))+ dst2;
        % if this cost is less than before than update
         if new_distance < dist(I,2)
            dist(I,2)=new_distance;
            parent(I,2)=u(i);
        end
    end
    dst =  dist(I,2);    
end

