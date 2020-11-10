close all
clear

% potential field
x = 0.01:0.1:10;
r = 1;
a = 1;
y = max(-a*log(x/r),0).^2;
dy = 2*a^2*log(x/r)./x;
plot(x,y)

% reciprocal barrier function
figure
b = -log(x./(1+x));
plot(x,b)
dh = (x+x.^2)/log(x./(1+x));
plot(x,dh)
t = 0:0.1:100;
h0 = 1;
h = 1./((exp(sqrt(2*t+log((h0+1)/h0)^2)))-1);
plot(t,h)

% inverse type
figure
b = 1./x;
plot(x,b)
t = 0:0.1:100;
h0 = 1;
h = 1./sqrt(2*t+(1/(h0^2)));
plot(t,h)