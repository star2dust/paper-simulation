function drawRoute(method,startnode,endnode,exnodelocation,exnodIndex,exunedges,rt,Cost)
% draw graph nodes and selected way.
import pathplanning.*
ns=size(exnodelocation,1);
map_definition();
hold on;

title([ method '  Cost:' num2str(Cost)]);
sn=exnodIndex(startnode);
en=exnodIndex(endnode);
plot(exnodelocation(1:ns,1),exnodelocation(1:ns,2),'b*');
plot(exnodelocation(sn,1),exnodelocation(sn,2),'rx','markersize',10);
plot(exnodelocation(en,1),exnodelocation(en,2),'rx','markersize',10);

for i=1:2:size(exunedges,1)    
    x1=exnodelocation(exunedges(i,1),1);
    x2=exnodelocation(exunedges(i,2),1);   
    y1=exnodelocation(exunedges(i,1),2);
    y2=exnodelocation(exunedges(i,2),2);  
    line([x1;x2],[y1;y2],'linewidth',0.1);    
end

for i=1:length(rt)-1
    x1=exnodelocation(exnodIndex(rt(i)),1);
    x2=exnodelocation(exnodIndex(rt(i+1)),1);   
    y1=exnodelocation(exnodIndex(rt(i)),2);
    y2=exnodelocation(exnodIndex(rt(i+1)),2);  
    line([x1;x2],[y1;y2],'color','r','linewidth',2);
end
hold off