function output = fcn_TrackingError(u)
global nodenum neighborMat stressMat dim leaderNum

p_all = u; %reshape(u,dim,nodenum);

p_ell=p_all(1:dim*leaderNum);
p_f=p_all(dim*leaderNum+1:end);
stressMat_ff=stressMat(leaderNum+1:end,leaderNum+1:end);
stressMat_fl=stressMat(leaderNum+1:end,1:leaderNum);
delta_pf=p_f+inv(kron(stressMat_ff,eye(dim)))*kron(stressMat_fl,eye(dim))*p_ell;
output=norm(delta_pf);