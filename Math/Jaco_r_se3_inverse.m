function [ y ] = Jaco_r_se3_inverse( E )

e1=E(1:3);
e2=E(4:6);

j1=jacor_inverse(e1);
Qr2=Kr( e1, e2 );


y= [  j1   zeros(3,3);...
     -j1*Qr2*j1  j1  ]; 



end

