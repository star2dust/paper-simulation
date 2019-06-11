function [nodez, nodepoly, nodezinpoly]= randnode(nodez,nodepoly,nodezinpoly,nodenum,nodeconfig,obstacle,range)
import iristools.*
% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% generate nodes outside obstacles
ctr=1; % counter
while (ctr<=nodenum)
    % generate random two number in range of map's border
    randlocation = [rand* (range.ub(1)-range.lb(1)) + range.lb(1);
        rand* (range.ub(2)-range.lb(2)) + range.lb(2)];
    % if this node is not inside any obstacle
    if ~inpolygon(randlocation(1),randlocation(2),obstacle.poly(1,:),obstacle.poly(2,:))
        % chech if inside a existing polytope
        if ~innodepoly(randlocation,nodepoly)
            % generate ploytope
            randpoly = polytope(obstacle.calc,randlocation,range);
            % check whether formation exists in a polytope
            [randz,fg] = formationP2z(randpoly,[randlocation',nodeconfig],range);
            if fg==1
                % add this location to nodelocation list
                nodez = [nodez;randz];
                nodepoly = [nodepoly,randpoly];
                nodezinpoly{polynum+ctr} = znum+ctr;
                %%%%%
%                 hold on
%                 plot(randlocation(1),randlocation(2),'go');
%                 plot(randz(1),randz(2),'r*');
%                 drawregion2d(randpoly,[],range);
%                 drawplat3d(randz);
%                 hold off
                %%%%%
                ctr=ctr+1;
            end
        end
    end
end
end

function flag = innodepoly(randlocation,nodepoly)
polynum = length(nodepoly);
count = 0;
for i=1:polynum
    if nodepoly(i).A*randlocation-nodepoly(i).b<=0
        count = count+1;
    end
end
if count==0
    flag = false;
else
    flag = true;
end
end