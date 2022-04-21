function out = circle_(cen, rad, varargin)
% Compute points used to plot a circle 
% An extension of circle function where the first point is in the direction of 'd'

opt.d = [1,0];
[opt,arglist] = tb_optparse(opt, varargin);

if nargout > 0
    % return now
    cen = cen(:)';
    p = circle(zeros(1,2), rad, arglist{:});
    thd = cart2pol(opt.d(1),opt.d(2));
    out = (SE2([cen,thd])*p)';
    out = [out;out(1,:)];
    return;
else
    % else plot the circle
    circle(cen, rad, arglist{:});
end

end