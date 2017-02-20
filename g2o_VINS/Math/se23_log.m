function [ y ] = se23_log( X )


R= X(1:3,1:3); 
v= X(1:3,4);
p= X(1:3,5);

theta = so3_log(R);

dv=jacor_inverse(-theta)*v;

dp=jacor_inverse(-theta)*p;

y=[theta; dv; dp];



end

