clear
clc

x= [0.2;0.3;-0.4; randn(3,1)];



dx=0.0001*ones(6,1);

%se3_log( se3_exp(x) ) -x
%SE3_x= se3_log( se3_group(se3_exp(x), se3_exp( Jacobian_Lie(x) * dx)  )) - x
%SE3_x= se3_log( se3_group(se3_exp(x), se3_exp(  dx)  )) - Jacobian_Lie_inverse( x )*dx- x
SE3_x= se3_log( se3_group(se3_exp(x), se3_exp(  dx)  )) - dx- x