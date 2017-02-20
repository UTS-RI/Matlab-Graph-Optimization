clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');
addpath('./VINS_DataGenerator/');
addpath('./Geometry/');

load VINS4.mat

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


% 
Measure_translation.inf=10000*eye(6);
T0= RobotState{1, 1}.pose; R0=T0(1:3,1:3); p0=T0(1:3,4);
T1= RobotState{1, 2}.pose; R1=T1(1:3,1:3); p1=T1(1:3,4);


Measure_translation.value=[R0'*R1 R0'*(p1-p0) ];

%[ Graph ] = AddNormalEdge( Graph, 'RelativePose3_Factor', 'Pose3', 'pose0',  'Pose3', 'pose1', Measure_translation );



%

%%% add IMU measurement
num_poses = 30;

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
Graph.Nodes.IMUbias.Values.(NodeArray{3,2})= [RobotState{i+1}.ba;RobotState{i+1}.bw];
Graph.Nodes.IMUbias.Values.(NodeArray{6,2})= [RobotState{i+2}.ba;RobotState{i+2}.bw];

%[ Graph ] = PerformGO( Graph ); % To get odometry

end
[ Graph ] = PerformGO( Graph );

Measurement_Vision_small = Measurement_Vision(1: (1+num_poses)*50,: );
[ LandmarkManager ] = Fn_LandmarkManager( Measurement_Vision_small );
Landmarks_values= Cal_triangulate( Graph.Nodes.Pose3.Values , LandmarkManager );
% %%% add Vision measurement
 i=1;

while( Measurement_Vision(i,1)<=  num_poses)
NodeArray=[];
pose_id = ['pose' num2str( Measurement_Vision(i,1) )];
pose_index = Measurement_Vision(i,1);
landmark_id =['landmark' num2str( Measurement_Vision(i,2) )];
j = Measurement_Vision(i,2);
Measurement.value.uv =  (Measurement_Vision(i,3:4))'  ;
Measurement.value.f = Landmarks_values.(landmark_id);
Measurement.value.id = landmark_id;
depth = norm(RGBD_data(i,3:5))*(1+randn*0.1);
Measurement.inf=eye(2)*(1/depth^2)*525^2;
NodeArray{1,1}='Pose3'; NodeArray{1,2}=pose_id;
[ Graph ]= AddComplexEdge(Graph, 'LinearVisionNoLandmark_Factor', NodeArray,  Measurement);

i=i+1;
end
[ Graph ] = PerformGO( Graph );

iter_max = 100;
Graph.Parameters.iter=1;

for i =1 : iter_max
    num_edges = size(Graph.Edges,2);
    
    for k = 1:num_edges
        if strcmp(Graph.Edges{k}.EdgeType, 'LinearVisionNoLandmark_Factor')
        landmark_id = Graph.Edges{k}.Measurement.value.id;
        Graph.Edges{k}.Measurement.value.f = Landmarks_values.(landmark_id);    
        end        
    end    
    [ Graph ] = PerformGO( Graph );
    Landmarks_values= Cal_triangulate( Graph.Nodes.Pose3.Values , LandmarkManager );
end





