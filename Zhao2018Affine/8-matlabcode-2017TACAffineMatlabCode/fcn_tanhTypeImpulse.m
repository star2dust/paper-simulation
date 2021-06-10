
function output=fcn_tanhTypeImpulse(time_start,mag,time_span,currentTime)
a=mag;
b=0.2;
if currentTime<time_start
    output=0;
elseif currentTime<time_start+time_span
    output=a*tanh(currentTime-time_start);
elseif currentTime>=time_start+time_span
    output=a*tanh(currentTime-time_start)-a*tanh(currentTime-time_start-time_span);
end