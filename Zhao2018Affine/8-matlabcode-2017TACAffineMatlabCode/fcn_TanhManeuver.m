function output=fcn_TanhManeuver(time_start, time_span, mag, currentTime)
% slide and then come back
a=mag;
b=0.2;
c1=-b*time_start;
c2=-b*(time_start+time_span);
output=-2*a*b^2*tanh(b*currentTime+c1)*(1-tanh(b*currentTime+c1)^2)...
    +2*a*b^2*tanh(b*currentTime+c2)*(1-tanh(b*currentTime+c2)^2);
