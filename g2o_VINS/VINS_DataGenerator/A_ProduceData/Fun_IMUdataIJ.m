function [ IMU_DataIJ ] = Fun_IMUdataIJ( i,j, Camera_timestep, IMU )

% i means pose i from 0 to Camera.num-1
% i <<  j  strictly

%Time_start=Time.start;

% ti=Fun_Camera_step2time( i, Time_start, Camera_timestep );
% tj=Fun_Camera_step2time( j, Time_start, Camera_timestep );

Iindex=floor(i*Camera_timestep/IMU.timestep)+1;   
Jindex=floor(j*Camera_timestep/IMU.timestep);

% if Jindex> size(IMU.gyro.Data,2)
%     Jindex=size(IMU.gyro.Data,2);
% end


IMU_DataIJ.gyro=IMU.gyro.Data(1:3, Iindex:Jindex );
IMU_DataIJ.acc=IMU.acc.Data(1:3, Iindex:Jindex );
IMU_DataIJ.timestep=IMU.timestep;
end


% function [ time ] = Fun_Camera_step2time( pointer, Time_start, Camera_timestep )
% 
% time= pointer*Camera_timestep+Time_start;
% 
% end

