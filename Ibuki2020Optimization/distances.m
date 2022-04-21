function dist = distances(graph,vert)
% The real distances between every two points in an undirected graph
robot_num = size(vert,1);
dist = zeros(robot_num);
for i=1:robot_num-1
    for j=i:robot_num
        if any(graph.edge_sort(2,graph.edge_sort(1,:)==i)==j)
            dist(i,j) = norm(vert(i,:)-vert(j,:));
        end
    end
end
dist = dist+dist';
end