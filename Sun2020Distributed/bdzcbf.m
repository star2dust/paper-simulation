function [aqp,bqp,zbf] = bdzcbf(graph,i,pos,ld,ud,aqp,bqp,vel)
% Bounded distance constraints use zeroing control barrier function
if nargin<8
    vel = zeros(size(pos));
    if nargin<7
        aqp = [];
        bqp = [];
    end
end
% extended K-class function
akcf = 1;
ekcf = @(h) akcf*h;
% bounded distance constraints
for j = neighbors(graph,i)
    zbf(j) = ((pos(i,:)-pos(j,:))*(pos(i,:)-pos(j,:))'-ld(i,j)^2)...
        *((pos(i,:)-pos(j,:))*(pos(i,:)-pos(j,:))'-ud(i,j)^2);
    cbfa = (2*(pos(i,:)-pos(j,:))*(pos(i,:)-pos(j,:))'-ld(i,j)^2-ud(i,j)^2)*(pos(i,:)-pos(j,:));
    cbfb = -ekcf(zbf(j))+cbfa*vel(j,:)';
    aqp = [aqp;cbfa];
    bqp = [bqp;cbfb];
end
end