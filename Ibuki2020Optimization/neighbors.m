function nbi = neighbors(graph,i)
% Find all neighbors of node i in an undirected graph
nbi = graph.edge_sort(2,graph.edge_sort(1,:)==i);
end