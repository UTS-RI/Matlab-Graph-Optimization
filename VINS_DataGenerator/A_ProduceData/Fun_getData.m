function [ IMU,Camera,NewVehicle, RGBD_data ] = Fun_getData( Time,IMU,Camera,Feature )

NewVehicle=struct;
NewVehicle.orientation=[];
NewVehicle.position=[];
NewVehicle.localvelocity=[];
NewVehicle.globalvelocity=[];
NewVehicle.gyro_bias_history=IMU.gyro.bias;
NewVehicle.acc_bias_history=IMU.acc.bias;

loadtime_IMU=Time.start;
loadtime_Camera=Time.start;

RGBD_data=[];


%while(loadtime_IMU<=Time.end) 
while((loadtime_IMU-Time.end)<=0.0000001)     
[ Vehicle ] = SimulatorVehicle( loadtime_IMU );
%%%%% Record IMU data  %%%%%%  
gyro_covs2= IMU.gyro.noise_cov^(1/2);   
IMU.gyro.Data=  [IMU.gyro.Data Vehicle.av+gyro_covs2*randn(3,1)/sqrt(IMU.timestep)+IMU.gyro.bias ];
acc_covs2= IMU.acc.noise_cov^(1/2);
IMU.acc.Data= [IMU.acc.Data Vehicle.accLocal+acc_covs2*randn(3,1)/sqrt(IMU.timestep)+IMU.acc.bias ];
%%%%%% Record IMU data %%%%%%%
%%%%% Record IMU biases and up these %%%%%%%
 IMU.gyro.bias=IMU.gyro.bias+IMU.gyro.bias_sigma*sqrt(IMU.timestep)*randn(3,1);
 IMU.acc.bias=IMU.acc.bias+IMU.acc.bias_sigma*sqrt(IMU.timestep)*randn(3,1);
 NewVehicle.gyro_bias_history=[NewVehicle.gyro_bias_history    IMU.gyro.bias];
 NewVehicle.acc_bias_history=[NewVehicle.acc_bias_history     IMU.acc.bias];
%%%%% Record IMU biases and up these %%%%%%%
%%%%% Record Vehicle Orientation, Position, Velocity_Global, Velocity_Local %%%%%%%
NewVehicle.orientation=[NewVehicle.orientation Vehicle.orientation];
NewVehicle.position=[NewVehicle.position Vehicle.position];
NewVehicle.localvelocity=[NewVehicle.localvelocity Vehicle.lv];
NewVehicle.globalvelocity=[NewVehicle.globalvelocity Vehicle.dp];
%%%%% Record IMU biases and up these %%%%%%%
IMU.timehistory=[IMU.timehistory loadtime_IMU];
loadtime_IMU=loadtime_IMU+IMU.timestep;
end



pointer=0;
while((loadtime_Camera-Time.end)<=0.00000001) 
[ Vehicle ] = SimulatorVehicle( loadtime_Camera );
temp=Vehicle.orientation'*(Feature.position-repmat(Vehicle.position,1,Feature.num));
Dxz=temp(1,:)./temp(3,:);
Dyz=temp(2,:)./temp(3,:);


Dxydz= [Dxz;Dyz];
fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;
K = [ fx 0; 0  fy];
UV = K*Dxydz+ repmat([cx0;cy0], 1, Feature.num ) +randn(2, Feature.num  ) ;



A1=ones( Feature.num,1)*pointer;
A2=1:1:Feature.num; A2=A2';
A3 = ( UV(1,:) )';     %A3=Dxz';
A4=  ( UV(2,:) )'; %Dyz';
localDataMatrix=[A1 A2 A3 A4];

temp_noise = temp+ 0.1*randn(3, Feature.num);

localRGBD=[A1 A2 temp_noise'];


%%%%% Pick out unexisting observations

kkk=0;
Real_local=[];
Real_localRGBD=[];
for  feature_id = 1: Feature.num
kkk=1;
distance = norm(temp(:,feature_id)  );


if localDataMatrix( feature_id , 3  )<0 || localDataMatrix( feature_id , 3  )>960 || distance >5
    kkk=0;
end
    
if localDataMatrix( feature_id , 4  )<0 || localDataMatrix( feature_id , 4  )>640  || distance >5
    kkk=0;
end    
    
if temp(3,  feature_id ) < 0
    kkk=0;
end


if  kkk==1  
Real_local=[Real_local;  localDataMatrix( feature_id , :  ) ];
Real_localRGBD=[Real_localRGBD; localRGBD( feature_id, : )  ];
end

end

%%%%% Pick out unexisting observations
RGBD_data=[RGBD_data;  Real_localRGBD  ];

Camera.Data=[Camera.Data; Real_local];

%Camera.Data.XZ=[Camera.Data.XZ Dxz'];
%Camera.Data.YZ=[Camera.Data.YZ Dyz'];

Camera.timehistory=[Camera.timehistory loadtime_Camera];    
loadtime_Camera=loadtime_Camera+Camera.timestep;

pointer=pointer+1;
end

Camera.NumOfFrames=pointer;


end

