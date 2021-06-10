function output=fcn_sin(time_start,time_span,currentTime)

% integral this function from 0 to time_span, it equals pi, so the agent
% rotate for pi angle
if currentTime<time_start || currentTime>time_start+time_span
    output=0;
else
    magnitude=pi^2/2/time_span;
    beta=pi/time_span;
    t=currentTime-time_start;
    output=magnitude*sin(beta*t);
end