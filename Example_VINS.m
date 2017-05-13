clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');
addpath('./VINS_DataGenerator/');

load VINS_Cubic.mat

[ Graph ] = InitializeGraph;


%%% add Prior
M_initial.value = RobotState{1, 1}.pose; 
M_initial.inf=100*eye(6);
[ Graph ] = AddUnaryEdge( Graph, 'PriorPose3_Factor', 'Pose3', 'pose0', M_initial );
Graph.Nodes.Pose3.Values.pose0 = RobotState{1, 1}.pose; 

Measurement.value.vel = RobotState{1, 1}.velocity ;
Measurement.value.bias = [RobotState{1, 1}.ba; RobotState{1, 1}.bw];
Measurement.inf=100*eye(9);
NodeArray=cell(2,2);
NodeArray{1,1}='Velocity3'; NodeArray{1,2}='v0';
NodeArray{2,1}='IMUbias'; NodeArray{2,2}='b0';
[ Graph ]= AddComplexEdge(Graph, 'PriorVelAndbias_Factor', NodeArray,  Measurement);
Graph.Nodes.Velocity3.Values.v0 = RobotState{1, 1}.velocity;
Graph.Nodes.IMUbias.Values.b0 = Measurement.value.bias;


%%% add IMU measurement
num_poses = 20;

for i = 0: num_poses-1
%%%%%%%% IMU part
NodeArray{1,1}='Pose3'; NodeArray{1,2}=['pose' num2str(i)];
NodeArray{2,1}='Velocity3'; NodeArray{2,2}=['v' num2str(i)];
NodeArray{3,1}='IMUbias'; NodeArray{3,2}=['b' num2str(i)];
NodeArray{4,1}='Pose3'; NodeArray{4,2}=['pose' num2str(i+1)];
NodeArray{5,1}='Velocity3'; NodeArray{5,2}=['v' num2str(i+1)];
NodeArray{6,1}='IMUbias'; NodeArray{6,2}=['b' num2str(i+1)];
Measurement_IMU_i = Measurement_IMU{i+1};

[ Graph ]= AddComplexEdge(Graph, 'IMU_Factor', NodeArray,  Measurement_IMU_i);

NodeArray_bias{1,1}='IMUbias'; NodeArray_bias{1,2}=['b' num2str(i)];
NodeArray_bias{2,1}='IMUbias'; NodeArray_bias{2,2}=['b' num2str(i+1)];
Measurement_IMU_change.value = [];
Measurement_IMU_change.inf = IMUbiasWalk_inf;
[ Graph ]= AddComplexEdge(Graph, 'IMUbias_Factor', NodeArray_bias,  Measurement_IMU_change);

Graph.Nodes.Pose3.Values.(NodeArray{1,2})=RobotState{i+1}.pose;
Graph.Nodes.Pose3.Values.(NodeArray{4,2})=RobotState{i+2}.pose;

Graph.Nodes.Velocity3.Values.(NodeArray{2,2})= RobotState{i+1}.velocity;
Graph.Nodes.Velocity3.Values.(NodeArray{5,2})= RobotState{i+2}.velocity;
Graph.Nodes.IMUbias.Values.(NodeArray{3,2})= [RobotState{i+1}.ba;RobotState{i+1}.bw];
Graph.Nodes.IMUbias.Values.(NodeArray{6,2})= [RobotState{i+2}.ba;RobotState{i+2}.bw];

%[ Graph ] = PerformGO( Graph ); % To get odometry

end
[ Graph ] = PerformGO( Graph );



% %%% add Vision measurement
 i=1;

while( RGBD_data(i,1)<=  num_poses)
NodeArray=[];
pose_id = ['pose' num2str( RGBD_data(i,1) )];
landmark_id =['landmark' num2str( RGBD_data(i,2) )];
Measurement.value=  (RGBD_data(i,3:5))'  ;
Measurement.inf=100*eye(3);

NodeArray{1,1}='Pose3'; NodeArray{1,2}=pose_id;
NodeArray{2,1}='Landmark3'; NodeArray{2,2}=landmark_id;
    
[ Graph ]= AddComplexEdge(Graph, 'RGBD_Factor', NodeArray,  Measurement);

Graph.Nodes.Landmark3.Values.(landmark_id )= Landmarks.(landmark_id);

i=i+1;    
end

[ Graph ] = PerformGO_DL( Graph );

% T_truth = RobotState{1, 49 }.pose;  R1 = T_truth(1:3,1:3); p1 = T_truth(1:3,4);
% T_es    = Graph.Nodes.Pose3.Values.pose48; R2 = T_es(1:3,1:3); p2 = T_es(1:3,4);
% 
% dR = R1'*R2;
% dp = R1'*(p2-p1);
% dX =[dR dp];
% norm(se3_log( dX ))



