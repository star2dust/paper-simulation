function [parent, route, cost]=dynamicpathplanning(Graph,Loc,Ind,startnode,endnode)
% calculate minimum costs path with recursive manner.
import pathplanning.*
route=[];
cost=0;
dist=[startnode 0];
parent=[startnode nan];
n=size(Graph,1);
deep=0;
% call main recursive function, this function will call itself and give us
% route
[ds dist parent ] = sp_dp(Graph, endnode, dist,parent,Loc,Ind,deep); 

Route=[];
Route=[endnode ];
next=1;
while Route(end)~=startnode && ~isnan(next)
    curr=Route(end);
    xx=find(parent(:,1)==curr);
    next=parent(xx,2);
    if ~isnan(next)        
        cost=cost+costcal(Loc(Ind(curr),:),Loc(Ind(next),:));
        Route=[Route next];
    end
end
if Route(end)==startnode
    route=Route;    
else
    route=[];
    cost=inf;
end

