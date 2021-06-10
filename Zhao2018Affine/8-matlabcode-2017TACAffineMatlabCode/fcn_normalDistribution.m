function output=fcn_normalDistribution(time_start,time_span,mag,currentTime)

mu=time_start+time_span/2;
sigma=time_span/8;

output=-mag*1/(sqrt(2*pi*sigma^2))*exp(-(currentTime-mu)^2/2/sigma^2);