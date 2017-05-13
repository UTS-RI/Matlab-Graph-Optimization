function [ Jprior_r0,ErrorVector_prior ] = Factor_Prior( x0, Priorx0 )

%addpath('../MathOperation/');

R0=x0(1:3,1:3);
p0=x0(1:3,5);

R_prior=Priorx0(1:3,1:3);
p_prior=Priorx0(1:3,5);


dR=R0*R_prior';
dP=p0-R0*R_prior'*p_prior;

theta1=so3_log( dR);
theta2= jacor_inverse( -theta1 )*(dP);
ErrorVector_prior=[theta1;theta2];


%adg_inverse=[ dR' zeros(3,3) ; -dR'*skew(dP) dR'  ];


Jprior_r0= [ dR' zeros(3,6); -dR'*skew(dP)  zeros(3,3)    dR'  ]  ;





end

