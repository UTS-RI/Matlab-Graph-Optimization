clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');
addpath('./Geometry/');
addpath('./VINS_DataGenerator/');

load VINS_cubic3.mat

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
%[ Graph ]= AddComplexEdge(Graph, 'PriorVelAndbias_Factor', NodeArray,  Measurement);
% Graph.Nodes.Velocity3.Values.v0 = RobotState{1, 1}.velocity;
% Graph.Nodes.IMUbias.Values.b0 = Measurement.value.bias;


% 
Measure_translation.inf=eye(6)*1e5;
T0= RobotState{1, 1}.pose; R0=T0(1:3,1:3); p0=T0(1:3,4);
T1= RobotState{1, 2}.pose; R1=T1(1:3,1:3); p1=T1(1:3,4);


Measure_translation.value=[R0'*R1 R0'*(p1-p0) ];

%[ Graph ] = AddNormalEdge( Graph, 'RelativePose3_Factor', 'Pose3', 'pose0',  'Pose3', 'pose1', Measure_translation );





%%% add IMU measurement 
num_poses = 49;

for i = 0: num_poses-1
%%%%%%%% IMU part
NodeArray=cell(2,6);
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

end
[ Graph ] = PerformGO_DL( Graph );



% %%% add Vision measurement
 i=1;

while( Measurement_Vision(i,1)<=  num_poses)
NodeArray=[];
pose_id = ['pose' num2str( Measurement_Vision(i,1) )];
pose_index = Measurement_Vision(i,1);
landmark_id =['landmark' num2str( Measurement_Vision(i,2) )];
Measurement.value=  (Measurement_Vision(i,3:4))'  ;
depth_rough = norm(RGBD_data(i,3:5))*(1+randn*0);
%Measurement.inf= (1/depth_rough)^2*eye(2)*(525^2);
Measurement.inf=eye(3)* (180/pi)^2*10;


NodeArray{1,1}='Pose3'; NodeArray{1,2}=pose_id;
NodeArray{2,1}='Landmark3'; NodeArray{2,2}=landmark_id;
    
T= RobotState{pose_index+1}.pose;
f = Landmarks.(landmark_id);
R=T(1:3,1:3); t = T(1:3,4);
z=R'*(f-t);

if isfield(Graph.Nodes, 'Landmark3' )
if isfield(Graph.Nodes.Landmark3.Values, landmark_id)
    kk =1 ;
else kk=0;
end
else kk=0;
end

if   1
   % Measurement.inf=eye(2);
[ Graph ]= AddComplexEdge(Graph, 'VisionTest_Factor', NodeArray,  Measurement);
if kk == 1
Graph.Nodes.Landmark3.Values.(landmark_id )= Landmarks.(landmark_id)+2*randn(3,1);
end

%pose = RobotState{pose_index+1}.pose;
%R = pose(1:3,1:3)*expm(skew( randn(3,1)*0.3));
%t = pose(1:3,4)+randn(3,1)*0.6;

%Graph.Nodes.Pose3.Values.(pose_id)=[ R t ];
end


i=i+1;    
end

S_max= find(Measurement_Vision(:,1)== num_poses , 1, 'last' );

[ LandmarkManager ] = Fn_LandmarkManager( Measurement_Vision(1: S_max, :) );
[ Landmarks_values ] = Cal_triangulate( Graph.Nodes.Pose3.Values , LandmarkManager );
Graph.Nodes.Landmark3.Values=Landmarks_values;

subplot(3,1,1);
PlotTrajectory( Graph );
title('Odometry And Triangulate');

%  load gns_Linear.mat;
%  Graph.Nodes = GraphNodes;
tic
[ Graph ] = PerformGO_LM( Graph );
toc
axis([-5 5 -5 5 -5 5]*8/5)
subplot(3,1,2)
PlotTrajectory( Graph );
title('Optimization');
axis([-5 5 -5 5 -5 5]*8/5)

subplot(3,1,3)
PlotTrajectory_Ground( RobotState, num_poses );
axis([-5 5 -5 5 -5 5]*8/5)




