function [map, nodelocation]= generate_node(map,nnode)

% merge vertices of all obstacle
obsx=map.pgx{1};
obsy=map.pgy{1};
for i=2:length(map.pgx)
    obsx=[obsx NaN map.pgx{i}];
    obsy=[obsy NaN map.pgy{i}];
end
map.obsx=obsx;
map.obsy=obsy; 
% set nodelocation to all zero
nodelocation=zeros(nnode,2);
% generate nodes
n=1;
while (n<=nnode)
    % generate random two number in range of map's border
    rx=rand* (map.xrange(2)-map.xrange(1)) + map.xrange(1);
    ry=rand* (map.yrange(2)-map.yrange(1)) + map.yrange(1);
    state=0;
    % if this node is not inside any obstacle
    if ~inpolygon(rx,ry,obsx,obsy)
        % add this location to nodelocation list
        nodelocation(n,1)=rx;
        nodelocation(n,2)=ry;
        n=n+1;
    end
end
hold on;
plot(nodelocation(:,1),nodelocation(:,2),'r*');
hold off;