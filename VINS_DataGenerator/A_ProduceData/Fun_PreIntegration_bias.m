function [ value, inf ] = Fun_PreIntegration_bias( IMU_DataIJ, IMUsettings, accbias_es, gyrobias_es )

addpath('MathOperation/');


N=blkdiag( IMUsettings.gyro_noise_cov, IMUsettings.acc_noise_cov, zeros(3,3)   );



 [ value, inf ] = rk4_manifold_mod( IMU_DataIJ, N, accbias_es, gyrobias_es   );

 
NN = size(IMU_DataIJ.gyro,2);

 
%J(1:3,1:3)=StrictlyRotationMatrix(J(1:3,1:3));


end


function R=StrictlyRotationMatrix(M)

c1=M(1:3,1);
c2=M(1:3,2);

c1=c1/norm(c1);
c2=c2-(c2'*c1)*c1;
c2=c2/norm(c2);
c3=skew(c1)*c2;

R=[c1 c2 c3];


end

