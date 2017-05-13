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
Graph.Fixed.IDname.pose0 =1; 
Graph.Fixed.IDname.pose1 =1;


num_poses = 49;

ss=1;
while( Measurement_Vision(ss,1)<=  num_poses)

    ss = ss+1;
    
end

i=1;

 
 
while( Measurement_Vision(i,1)<=  num_poses)
NodeArray=[];
pose_id = ['pose' num2str( Measurement_Vision(i,1) )];
pose_index = Measurement_Vision(i,1);
landmark_id =['landmark' num2str( Measurement_Vision(i,2) )];
Measurement.value=  (Measurement_Vision(i,3:4))'  ;
depth_rough = norm(RGBD_data(i,3:5))*(1+randn*0);
%Measurement.inf= (1/depth_rough)^2*eye(2)*(525^2);
Measurement.inf=eye(3);

NodeArray{1,1}='Pose3'; NodeArray{1,2}=pose_id;
NodeArray{2,1}='Landmark3'; NodeArray{2,2}=landmark_id;
    

if isfield(Graph.Nodes, 'Landmark3' )
if isfield(Graph.Nodes.Landmark3.Values, landmark_id)
    kk =1 ;
else kk=0;
end
else kk=0;
end

num_landmark_id = Measurement_Vision(i,2);
num_ob_id = sum(Measurement_Vision(1:ss-1,2)==num_landmark_id);

if num_ob_id>=2 
if   1
[ Graph ]= AddComplexEdge(Graph, 'VisionTest_Factor', NodeArray,  Measurement);
if kk == 1
Graph.Nodes.Landmark3.Values.(landmark_id )= Landmarks.(landmark_id)+1.3*randn(3,1);

end
end
pose = RobotState{pose_index+1}.pose;
R = pose(1:3,1:3)*expm(skew( randn(3,1)*.1));
t = pose(1:3,4)+randn(3,1)*1.2;

Graph.Nodes.Pose3.Values.(pose_id)=[ R t ];
end


i=i+1;    
end
Graph.Nodes.Pose3.Values.pose0 = RobotState{1}.pose;
Graph.Nodes.Pose3.Values.pose1 = RobotState{2}.pose;


S_max= find(Measurement_Vision(:,1)== num_poses , 1, 'last' );

[ LandmarkManager ] = Fn_LandmarkManager( Measurement_Vision(1: S_max, :) );
[ Landmarks_values ] = Cal_triangulate( Graph.Nodes.Pose3.Values , LandmarkManager );
%Graph.Nodes.Landmark3.Values=Landmarks_values;










subplot(1,3,1);
PlotTrajectory( Graph );
title('Odometry And Triangulate');

%  load gns_Linear.mat;
%  Graph.Nodes = GraphNodes;
tic
[ Graph ] = PerformGO_LM( Graph );
toc
axis([-5 5 -5 5 -5 5]*8/5)
subplot(1,3,2)
PlotTrajectory( Graph );
title('Optimization');
axis([-5 5 -5 5 -5 5]*8/5)

subplot(1,3,3)
PlotTrajectory_Ground( RobotState, num_poses );
axis([-5 5 -5 5 -5 5]*8/5)




