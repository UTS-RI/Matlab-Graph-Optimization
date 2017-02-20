function [ y ] = se3_exp( x )

x1=x(1:3);
x2=x(4:6);

R= so3_exp(x1);
p= jaco_r(-x1)*x2;


y=[R p];




end

