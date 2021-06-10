function stressMat=fcn_StressMatrix(d,n,m,P,adjMat)
% incidence matrix
H=zeros(m,n);
pointer=1;
for i=1:n
    for j=i+1:n
        if adjMat(i,j)~=0
            H(pointer,i)=-1;
            H(pointer,j)=1;
            pointer=pointer+1;
        end
    end
end
if (pointer-1)~=m
    display('Shiyu Error: m is set wrongly')
    return
end

%
Pbar=[ones(n,1),P];
[U,S,V]=svd(Pbar);
U2=U(:,d+1+1:end);
Q=U2';

%% Plot the formation
hold on; axis equal
% plot edges
for i=1:n
    for j=1:n
        if adjMat(i,j)~=0
            pi=P(i,:);
            pj=P(j,:);
            line([pi(1),pj(1)], [pi(2),pj(2)], 'linewidth', 2, 'color', 'b');
        end
    end
end
% plot nodes
for i=1:n
    pi=P(i,:);
    plot(pi(1), pi(2), 'o', ...
        'MarkerSize', 15,...
        'linewidth', 2,...
        'MarkerEdgeColor', 'r',...
        'markerFaceColor', 'white');
    text(pi(1), pi(2), num2str(i),...
        'color', 'r', 'FontSize', 13, 'horizontalAlignment', 'center', 'FontName', 'times');
end
% numbering edges
pointer=1;
for i=1:n
    for j=i+1:n
        if adjMat(i,j)~=0
            pi=P(i,:);
            pj=P(j,:);
            text((pi(1)+pj(1))/2, (pi(2)+pj(2))/2, num2str(pointer),...
                'color', 'k', 'FontSize', 12, 'horizontalAlignment', 'center');
            pointer=pointer+1;
        end
    end
end
%%
E=zeros(d*n,m);
for i=1:n
    hi=H(:,i);
    E((i-1)*d+1:(i-1)*d+d,:)=P'*H'*diag(hi);
end
[U,S,V]=svd(E);
% the columns of B are the basis to calculate omega
B=V(:,rank(S)+1:end); 
% if B is just one dimensional, then it is w, 
% otherwise, its columns are the basis of w: w=x1b1+x2b2+...
num=size(B,2);

%% if num>1, need LMI!
M=zeros(n-d-1,n-d-1,num);
for i=1:num
    bi=B(:,i);
    M(:,:,i)=Q*H'*diag(bi)*H*Q';
end
setlmis([]);
LMI=newlmi;
if num==0
    display('Shiyu info: B has 0 column; No self-stress')
    return;
elseif num==1
    display('Shiyu info: B has 1 column')
    x1=lmivar(1,[1 0]);
    % inequality: I<mu*(M1x1+M2x2+...)
    lmiterm([-LMI 1 1 x1],M(:,:,1),1); % right hand side
    LMIsys=getlmis;
    [tmin,x] = feasp(LMIsys,[0 0 1 0 0]);
elseif num==2
    display('Shiyu info: B has 2 columns')
    x1=lmivar(1,[1 0]);
    x2=lmivar(1,[1 0]);
    % inequality: I<mu*(M1x1+M2x2+...)
    lmiterm([-LMI 1 1 x1],M(:,:,1),1); % right hand side
    lmiterm([-LMI 1 1 x2],M(:,:,2),1); % right hand side
    LMIsys=getlmis;
    [tmin,x] = feasp(LMIsys,[0 0 1 0 0]); % the third para is ||x||<1
elseif num>2
    display('Shiyu Error: B has more than 2 columns')
    return;
end
%% calculate omega and stress matrix
omega=B*x;
omega=omega/norm(omega);
stressMat=H'*diag(omega)*H;
if max(eig(stressMat))>0
    omega=-omega;
    stressMat=-stressMat;
end
% omega
% stressMat
% eig(stressMat)
if max(eig(stressMat))>10^-3
    display('Shiyu Error: Stress Matrix is not PSD')
end
