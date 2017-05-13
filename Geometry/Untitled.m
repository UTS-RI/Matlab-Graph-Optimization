clc
clear


n1= randn(3,1); n1 = n1/norm(n1);
n2= randn(3,1); n2 = n2/norm(n2);

x1 = [n1;randn];x2 = [n2;randn];


y=logmap_parallax(x1,x2);


so3_exp(null(n2')*y(1:2))*n2 - n1
