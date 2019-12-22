function [route] = astar(exbigraph,exbiloc,startnode,endnode)
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


% pre computed heuristic dist
% every nodes euristic cost save on matirx
for i=1:n
    HCost(i,1)=costcal(Loc(endnode,:),Loc(i,:));
end

i=1;
% if the endnode is found or all possibility is finished
while i<n & curr ~=endnode    
    i=i+1;
    T=1:n;
    T(Tabu)=[];  
    % find all possibility from current node
    I=find(graph(curr,T)==1);
    % find all possibility's cost from current to concerned node
    cost=costcal(Loc(curr,:),Loc(T(I),:)) + Cost(curr);
    % find if calculated cost is less than the way before explored
    J=find(Cost(T(I))>cost);
    % if new calculated is less than before one then set less one
    Cost(T(I(J)))=cost(J);  
    % if new calculated way is less than the before one than set new route
    % too
    for j=1:length(J)
        Route{T(I(J(j)))}=[cRoute T(I(J(j)))];        
    end  
    % find minimum of normal cost plus heuristic cost of possible node
    [mn cr]=min(Cost(T)+HCost(T));
    if isinf(mn)
        i=n;
    end
    % set minimum total cost's node as current node
    curr=T(cr);    
    cRoute=Route{curr};
    % add current node to tabu list to prevent loops
    Tabu=[Tabu curr];
end
% if current node is equal endnode means route is found
if curr ==endnode
    route=cRoute;
else % i not means there is no way
    route=[];
end
