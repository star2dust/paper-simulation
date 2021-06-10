function acceleration=fcn_SlowDownSpeedUp(time_start, time_span, currentTime, distance_change)

% first slow down and then speed up, the acceleration at first is negative
% and then positive. The velocities at the begning and end are the same,


mu=time_start+time_span/2;
sigma=time_span/8;

% derivative of a normal distribution: first slow down then speed up
acceleration=-distance_change*1/(sqrt(2*pi*sigma^2))*exp(-(currentTime-mu)^2/2/sigma^2)*(-currentTime+mu)/sigma^2;