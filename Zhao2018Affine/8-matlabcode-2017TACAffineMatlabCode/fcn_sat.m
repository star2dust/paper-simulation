function y=fcn_sat(x)
beta=5;
y=x;
for i=1:size(x,1)
    if abs(x(i))<beta
        y(i)=x(i);
    elseif x(i)>0
        y(i)=beta;
    else
        y(i)=-beta;
    end
end