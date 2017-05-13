function [ y ] = dDirectProject( x )

d = norm(x);

y = 1/d*eye(3)- x*x'/d^3;

end

