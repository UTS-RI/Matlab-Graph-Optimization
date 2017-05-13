function [ IMU, Camera, Time ] = Fun_initilization


IMU=struct;
Camera=struct;



IMU.frequency=100; %Hz
IMU.timestep=0.01;
IMU.timehistory=[];
%IMU.gyro.bias=[-0.000728023252950725;-0.000617546436714006;0.000302369770010283];
%IMU.gyro.bias_SigmaDynamics=0.0004;
%IMU.gyro.bias_CovDynamics=IMU.gyro.bias_SigmaDynamics^2*eye(3);
%IMU.gyro.bias_invCovDynamics=inv(IMU.gyro.bias_CovDynamics);
gyro_noise_sigma=0;%0.0007; % rad/s
IMU.gyro.noise_cov= gyro_noise_sigma^2*eye(3);
IMU.gyro.noise_inf=inv(IMU.gyro.noise_cov);
IMU.gyro.Data=[];  % will be 3 * NumberOfTimeSteps

IMU.gyro.bias=[0;0;0];
IMU.gyro.bias_sigma= 0 ;%0.0004;


IMU.acc.bias=[0;0;0];
IMU.acc.bias_sigma= 0;%0.012;


acc_noise_sigma= 0;%0.019; % m/(s^2)
IMU.acc.noise_cov=acc_noise_sigma^2*eye(3);
IMU.acc.noise_inf=inv(IMU.acc.noise_cov);
IMU.acc.Data=[];      % will be 3 * NumberOfTimeSteps

Camera.frequency=2.5; % Hz
Camera.timestep=1/Camera.frequency;
Camera.noise_sigma=1/315;   % 1 pixel noise, focal length is 315 pixel
Camera.Data=[];
%Camera.Data.XZ=[];
%Camera.Data.YZ=[];
Camera.timehistory=[];


Time.start=0;
Time.end=20;



end

