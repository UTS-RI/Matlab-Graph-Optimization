function [ SparseJacobian_values, FullErrorVector ] = Fun_GetJacobianError( LinearizedPoint, IMUfactorMeasurement, Camera,  SparseJacobian_values , FullErrorVector, Feature )

num.robotstate=Camera.NumOfFrames;
num.feature=Feature.num;
num.observation=size(Camera.Data,1);
FrameTime=Camera.timestep;


% Frior Factor
Prior=LinearizedPoint.robot(1:3,1:5);
x0=LinearizedPoint.robot(1:3, 1:5);
[ Jprior_r0,ErrorVector_prior ] = Factor_Prior( x0, Prior );
FullErrorVector(1:6,1)=ErrorVector_prior;
SparseJacobian_values(1:54)  = Jprior_r0(:);

% IMU Factor
for imu_index=0:num.robotstate-2
   xi=LinearizedPoint.robot( 1+3*imu_index :3+ 3*imu_index  ,1:5);
   xj=LinearizedPoint.robot( 1+3*imu_index+3 :3+ 3*imu_index+3  ,1:5);
   IMU_preintegrationIJ=IMUfactorMeasurement( 1+3*imu_index :3+ 3*imu_index,1:5);
   
   [ Jimu_ri, Jimu_rj, ErrorVector_imu_rirj ] = Factor_IMU( xi, xj, IMU_preintegrationIJ, FrameTime );
   FullErrorVector(  6 + 9*imu_index+ 1: 6 +9*imu_index+ 9         )=ErrorVector_imu_rirj;
   J_imu=[ Jimu_ri, Jimu_rj ];
   SparseJacobian_values(  54+ 2*imu_index*9^2+1 : 54+ 2*imu_index*9^2+81*2      )    =J_imu(:);
   
end

% Vision Factor
for vision_index=1:num.observation
     Ri_index=Camera.Data(vision_index,1);
     fk_index=Camera.Data(vision_index,2);
     
     xi=LinearizedPoint.robot( 1+3*Ri_index :3+ 3*Ri_index  ,1:5);
     fk_index_row= find( LinearizedPoint.feature( 4,: )== fk_index ) ;
     fk=LinearizedPoint.feature( 1:3, fk_index_row  );
     Vision_ik=Camera.Data(vision_index, 3:4);
    
     [ Jvision_ri, Jvision_fk, ErrorVector_vision_ri_fk ] = Factor_Vision( xi, fk, Vision_ik );
    % ErrorVector_vision_ri_fk
     J_sub=[Jvision_ri, Jvision_fk];
     FullErrorVector( 6+9*(num.robotstate-1)+ 2*(vision_index-1)+1   : 6+9*(num.robotstate-1)+ 2*(vision_index-1)+2 ) =ErrorVector_vision_ri_fk;
     
     index=54+ 2*( num.robotstate-1 )*9^2+24*(vision_index-1)+1 : 54+ 2*(num.robotstate-1)*9^2+24*(vision_index-1)+24;
     SparseJacobian_values(  index  )=J_sub(:); 
end



end

