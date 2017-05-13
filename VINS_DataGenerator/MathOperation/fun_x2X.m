function [ X ] = fun_x2X( x )


X=x;

X(1:3,4)=x(1:3,1:3)*x(1:3,4);

end

