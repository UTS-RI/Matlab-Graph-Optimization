function [ Measurement_IMU, IMUbiasWalk_inf] = Fun_DoPreintegration( LinearizedPoint, IMU, Camera )




Measurement_IMU=cell( 1, Camera.NumOfFrames-1);

for i=0:Camera.NumOfFrames-2

 accbias_es=LinearizedPoint.robot( 3*i+1 : 3*(i+1), 7  );
 gyrobias_es=LinearizedPoint.robot( 3*i+1 : 3*(i+1), 8  );
   
    
 [ IMU_DataIJ ] = Fun_IMUdataIJ( i,i+1, Camera.timestep, IMU );
 
 IMUsettings=struct;
 IMUsettings.gyro_noise_cov= IMU.gyro.noise_cov;
 IMUsettings.acc_noise_cov=IMU.acc.noise_cov;
 IMUsettings.accbias_es=accbias_es;
 IMUsettings.gyrobias_es=gyrobias_es;
 IMUsettings.gyro.bias_cov= ( eye(3)* IMU.gyro.bias_sigma^2  );
 IMUsettings.acc.bias_cov=  ( eye(3)* IMU.acc.bias_sigma^2  );
 
%  accbias_es =zeros(3,1);
%  gyrobias_es = zeros(3,1);
 
 [ value, inf ] = Fun_PreIntegration_bias( IMU_DataIJ, IMUsettings,  accbias_es, gyrobias_es   );

 IMUbiasWalk_inf=  (1/Camera.timestep)*blkdiag( eye(3)*(1/(IMU.acc.bias_sigma^2 )) , eye(3)*(1/(IMU.gyro.bias_sigma^2) )  ) ;
 
 
 value.dt = Camera.timestep;
 value.bw=gyrobias_es;
 value.ba=accbias_es;
 
 Measurement_IMU{i+1}.value=value;
 Measurement_IMU{i+1}.inf=inf;
 Measurement_IMU{i+1}.fromPose=['pose' num2str(i)];
 Measurement_IMU{i+1}.toPose=['pose' num2str(i+1)];
end




end

