function n = normby(x,s,p)
% Calculate vector norms for a matrix by row (1) or column (2)
%
%     This function returns the norm of matrix x.
%     s = 1 means by row, s = 2 means by column
%
%     n = norm(x,s)
%     n = norm(x,s,p)
if isempty(x)
    n = [];
else
    for i=1:size(x,s)
        if s==1
            if nargin<3
                n(i,:) = norm(x(i,:));
            else
                n(i,:) = norm(x(i,:),p);
            end
        elseif s==2
            if nargin<3
                n(:,i) = norm(x(:,i));
            else
                n(:,i) = norm(x(:,i),p);
            end
        else
            error('unknown argument');
        end
    end
end