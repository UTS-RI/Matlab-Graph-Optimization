function [ Settings, Camera, RealVehicle, Feature, IMU, RGBD_data] = Fun_ProduceData

addpath('MathOperation/');
addpath('A_ProduceData/');


[IMU, Camera, Time]=Fun_initilization;
[ Feature ] = Fun_generateFeatures;
[ IMU,Camera,RealVehicle, RGBD_data ] = Fun_getData( Time,IMU,Camera,Feature );




FrameTime=Camera.timestep;
NumOfFrames=Camera.NumOfFrames;

IndexFeatures=unique(Camera.Data(:,2));
NumOfFeatures=size(IndexFeatures,1);

Settings.FrameTime=FrameTime;
Settings.NumOfFrames=NumOfFrames;
Settings.NumOfFeatures=NumOfFeatures;
Settings.BS=floor(Camera.timestep/IMU.timestep);


end

