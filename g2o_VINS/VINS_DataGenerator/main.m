clc
clear
close all

addpath('MathOperation/');
addpath('A_ProduceData/');
addpath('Factors/');
addpath('OptimizationProcess/');

% Produce necessary settings&measurements for optimization, and ground truth for comparison   
tic
[ Settings,  Camera, RealVehicle, Feature, IMU, RGBD_data] = Fun_ProduceData;
%%% Get Initial Estimate
LinearizedPoint=struct;
[ LinearizedPoint.robot ] = Fun_InitialEstimate( RealVehicle, Settings );      
LinearizedPoint.feature= [Feature.position; 1:Feature.num];

%%%%%%%%%%%%%%%%%%%%%%%% Preintegration based on the current bias estimate
[ Measurement_IMU, IMUbiasWalk_inf ]   =Fun_DoPreintegration( LinearizedPoint, IMU, Camera   );
  Measurement_Vision = Camera.Data;                        
toc


num_robot = size(LinearizedPoint.robot, 1)/3;
RobotState = cell(1 ,num_robot );

for i=1:num_robot
RobotState{i}.id = ['pose' num2str(i-1)];
RobotState{i}.pose = LinearizedPoint.robot(3*i-2:3*i,1:4 ); 
RobotState{i}.velocity = LinearizedPoint.robot(3*i-2:3*i,5 );
RobotState{i}.ba= LinearizedPoint.robot(3*i-2:3*i,7 );
RobotState{i}.bw= LinearizedPoint.robot(3*i-2:3*i,8 );
end

for i=1: Feature.num
    landmark_id = ['landmark' num2str(i)];
    Landmarks.(landmark_id) = Feature.position( 1:3, i);
    
end

clear Camera Feature GroundTruth i IMU landmark_id LinearizedPoint num_robot RealVehicle Settings