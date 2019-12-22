function fig = plotx2T(x,c)
import rvctools.*
hold on
T = x2T(x);
[R,t] = tr2rt(T);
fig= quiver3(t(1)*ones(1,3),t(2)*ones(1,3),t(3)*ones(1,3),R(1,:)/3,R(2,:)/3,R(3,:)/3,c);
hold off
end