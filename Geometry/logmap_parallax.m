function [ y ] = logmap_parallax( x1, x2 )
%  x1 - x2


y = zeros(3,1);

 n1 = x1(1:3); theta1 = x1(4);
 n2 = x2(1:3); theta2 = x2(4);


angle = acos(n1'*n2);

if angle > 0.000001
n3 = skew(n2)*n1;
n3 = n3/norm(n3);
A = null(n2'); 
y(1:2) = inv(A(1:2,1:2))*n3(1:2)*angle;
else
    
    y(1:2) = [0;0];
end
y(3) = theta1 - theta2;

end

