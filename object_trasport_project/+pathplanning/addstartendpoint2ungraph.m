function [extungraph,exnodelocation,exunedges ]=addstartendpoint2ungraph(map,undirectedGraph,nodelocation,unedges,startp,endp)
% this function add two new nodes on undirected graoh.
% not need to construct again, just add new connections.

extungraph=undirectedGraph;
exnodelocation=nodelocation;
nnode=size(undirectedGraph,1);
% new coming nodes take place at new 2 order.
% it it has 100 node then 101 and 102 nodes are new adding node
exnodelocation(nnode+1,:)=startp;
exnodelocation(nnode+2,:)=endp;
exunedges=unedges;

for i=1:nnode
    x1=[nodelocation(i,1);startp(1)];
    y1=[nodelocation(i,2);startp(2)];
    % if any nodes and first newcoming nodes have a connection or not.
    [xi,yi] = polyxpoly(x1,y1,map.obsx,map.obsy);
    % if there is not any intersection with obstacle
    if length(xi)==0
        % add this connection to graph
        extungraph(nnode+1,i)=1;
        extungraph(i,nnode+1)=1;
        exunedges=[exunedges;i nnode+1;nnode+1 i];
    end
end
for i=1:nnode+1    
    x1=[exnodelocation(i,1);endp(1)];
    y1=[exnodelocation(i,2);endp(2)];
    % if any nodes and second newcoming nodes have a connection or not.
    [xi,yi] = polyxpoly(x1,y1,map.obsx,map.obsy);
     % if there is not any intersection with obstacle
    if length(xi)==0
        extungraph(nnode+2,i)=1;
        extungraph(i,nnode+2)=1;
        exunedges=[exunedges;i nnode+2;nnode+2 i];
    end
end