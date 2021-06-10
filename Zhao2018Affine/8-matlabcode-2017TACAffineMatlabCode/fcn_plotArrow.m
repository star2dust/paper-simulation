function fcn_plotArrow(pTail, g, color, linewidth)

length=0;
headSize=0.8;

% plot from tail to head
pHead=pTail+g*length;
line([pHead(1),pTail(1)], [pHead(2),pTail(2)], 'linewidth', linewidth, 'color', color);
% plot head arrow
tht=30/180*pi;
R=[cos(tht), -sin(tht);
    sin(tht), cos(tht)];
pArrow1=pHead-R*g*headSize;
pArrow2=pHead-R'*g*headSize;
line([pHead(1),pArrow1(1)], [pHead(2),pArrow1(2)], 'linewidth', linewidth, 'color', color);
line([pHead(1),pArrow2(1)], [pHead(2),pArrow2(2)], 'linewidth', linewidth, 'color', color);
