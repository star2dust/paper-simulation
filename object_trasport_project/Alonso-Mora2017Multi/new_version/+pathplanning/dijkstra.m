function [Route, Cost] = dijkstra(exbigraph,exbiloc,startnode)
% calculate route for given graph and starting point.
% this function finds all targets so you do not need to support end point
import pathplanning.*
graph=exbigraph;
Loc=exbiloc;
n=size(graph,1);
Cost=inf(n,1);
curr=startnode;
Route=cell(n,1);
Cost(curr)=0;
Tabu=[curr];
cRoute=[curr];
i=1;
% for number of nodes
while i<n
    i=i+1;
    % take all possible node list
    T=1:n;
    % delete already visited node
    T(Tabu)=[];   
    % find connected node from current node which are not visited yet
    I=find(graph(curr,T)==1);
    % calculate connected nodes euclid distance from current node and add
    % current nodes cost
    cost=costcal(Loc(curr,:),Loc(T(I),:)) + Cost(curr);
    % if new finding coset is less than old cost then replace
    J=find(Cost(T(I))>cost);
    Cost(T(I(J)))=cost(J);   
    % update new comming node rote if the new cost is more efficient
    for j=1:length(J)
        Route{T(I(J(j)))}=[cRoute T(I(J(j)))];        
    end  
    % find minimum costed node which is not visited yet in this iteration
    [mn cr]=min(Cost(T));
    if isinf(mn)
        i=n;
    end
    curr=T(cr);
    % select selected route as forbitten route to not to select this nodes
    % again
    cRoute=Route{curr};
    Tabu=[Tabu curr];
end

