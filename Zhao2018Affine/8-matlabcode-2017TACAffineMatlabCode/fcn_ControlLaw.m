function output = fcn_ControlLaw(u)
global nodenum neighborMat stressMat dim leaderNum ifLeaderFollower kp kv
u_all = reshape(u,dim,3*nodenum);
p_all=u_all(:,1:nodenum);
v_all=u_all(:,nodenum+1:2*nodenum);
v_dot_all_previous=u_all(:,2*nodenum+1:end);
%output
v_dot_all=zeros(dim,nodenum);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:nodenum % for agent i
    for j=1:nodenum % for the neighbors of agent i
        if neighborMat(i,j)==1 && j~=i% the agent j is the neighbor of agent i
            omegaij=stressMat(i,j);
            % control law 1: no accelaration
            v_dot_all(:,i)=v_dot_all(:,i) - kp*omegaij*(p_all(:,i)-p_all(:,j)) - kv*omegaij*(v_all(:,i)-v_all(:,j));
        end
    end
%     v_dot_all(:,i)=fcn_sat(v_dot_all(:,i));
end

%% based on control law 1, compute control law 2
ifAcceleration=1;
if ifAcceleration==1
    % compute sum omega_ij
    omegaSumAll=zeros(nodenum,1);
    for i=1:nodenum
        for j=1:nodenum
            if neighborMat(i,j)==1 && j~=i
                omegaSumAll(i)=omegaSumAll(i)+stressMat(i,j);
            end
        end
    end
    % add the acceleration term
    for i=1:nodenum
        for j=1:nodenum 
            if neighborMat(i,j)==1 && j~=i
                omegaij=stressMat(i,j);
                v_dot_all(:,i)=v_dot_all(:,i)+omegaij*v_dot_all_previous(:,j); 
            end
        end
        v_dot_all(:,i) = v_dot_all(:,i)/omegaSumAll(i);
    end
end

%%
output=reshape(v_dot_all,dim*nodenum,1);
