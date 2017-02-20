function [ y ] = se3_log( x )
R= x(1:3,1:3);
p= x(1:3,4);


theta = so3_log(R);

dp=jacor_inverse(-theta)*p;


y=[theta; dp];


end

