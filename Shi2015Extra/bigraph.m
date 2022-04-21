function graph = bigraph(robot_num,vert_ref)
% Generate undirected graph for at most 8 agents

% topology graph
graph.incidence =   [-1,-1,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1;
                      1, 0, 0, 0,-1,-1, 0, 0, 0, 0, 0,-1, 0, 0, 0, 0, 0;
                      0, 1, 0, 0, 1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0, 0;
                      0, 0, 1, 0, 0, 1, 0,-1, 0,-1, 0, 0, 0, 0, 0, 0, 0;
                      0, 0, 0, 1, 0, 0, 1, 1, 0, 0,-1, 0,-1, 0, 0, 0, 0;
                      0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,-1,-1, 0, 0;
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0,-1, 0;
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1];  
% remove redundant nodes
num_remove = size(graph.incidence,1)-robot_num;
if num_remove>0
    for i=1:num_remove
        graph.incidence(:,graph.incidence(end,:)~=0) = [];
        graph.incidence(end,:) = [];
    end
end
% laplacian
graph.laplacian = graph.incidence*graph.incidence';
graph.adjacency = diag(diag(graph.laplacian))-graph.laplacian;
% edge set
edge_num = size(graph.incidence,2);
graph.edge_set = mod(reshape(find(graph.incidence~=0),2,edge_num),robot_num);
graph.edge_set(graph.edge_set==0) = robot_num;
% undirected graph
edge_set_sort = [graph.edge_set,[0,1;1,0]*graph.edge_set];
[~,sort_ind] = sort(edge_set_sort(1,:));
graph.edge_sort = edge_set_sort(:,sort_ind);
if nargin>1&&robot_num>4
    % vertex dimension
    vert_dim = size(vert_ref,2);
    % configuration matrix
    mat_P = [vert_ref,ones(robot_num,1)];
    % calculate stress matrix
    mat_H = graph.incidence';
    mat_E = [];
    for i=1:robot_num
        mat_E = [mat_E;mat_P'*mat_H'*diag(mat_H(:,i))];
    end
    [mat_U,~,~] = svd(mat_P);
    % the order in S: from big to small
    mat_U2 = mat_U(:,vert_dim+2:end);
    % calculate M
    null_E = null(mat_E);
    coef_E = zeros(size(null_E,2),1);
    for i=1:size(null_E,2)
        basis_M{i} = mat_U2'*mat_H'*diag(null_E(:,i))*mat_H*mat_U2;
    end
    % LMI solver
    setlmis([]);
    for i=1:size(null_E,2)
        coef_E(i) = lmivar(1,[1,0]);
        lmiterm([-1,1,1,coef_E(i)],1,basis_M{i});
    end
    lmi = getlmis;
    [~,csol] = feasp(lmi);
    for i=1:size(null_E,2)
        coef_E(i) = dec2mat(lmi,csol,coef_E(i));
    end
    coef_E = coef_E/norm(coef_E);
    mat_M = [basis_M{:}]*kron(coef_E,eye(size(basis_M{1},2)));
    % check if all(eig(U2'*H'*diag(z)*H*U2)>0)
    eig_M = eig(mat_M);
    if all(eig_M>0)
        graph.weight_set =  (null_E*coef_E)';
        edge_weight_sort = [graph.weight_set,graph.weight_set];
        graph.weight_sort = edge_weight_sort(sort_ind);
        graph.stress = mat_H'*diag(graph.weight_set)*mat_H;
        graph.stressnull = mat_P;
        disp('M is positive definite.')
    else
        error('M is not positive definite.')
    end
end
end