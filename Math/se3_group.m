function [ y ] = se3_group( X1, X2 )

C1=  [X1; 0 0 0 1];
C2=  [X2; 0 0 0 1];

C=C1*C2;


y=C(1:3,1:4);



end

