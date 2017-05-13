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
Graph.Fixed.IDname.pose0=1;  % fix the variable pose0 
Graph.Fixed.IDname.pose1=1;  % fix the variable pose1

Graph.Schur.Landmark3 = 1;   % perform schur docomposition for the Nodetype Landmark3


[ Graph2 ] = InitializeGraph;


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
      NodeArray{1,1}='Pose3'; NodeArray{1,2}=pose_id;
      NodeArray{2,1}='Landmark3'; NodeArray{2,2}=landmark_id;
      Measurement.value=  (Measurement_Vision(i,3:4))'  ;
      Measurement.inf=eye(3);
      [ Graph ]= AddComplexEdge(Graph, 'VisionTest_Factor', NodeArray,  Measurement);
      
      Measurement2.value=  (Measurement_Vision(i,3:4))'  ;
      Measurement2.inf=eye(2);      
      [ Graph2 ]= AddComplexEdge(Graph2, 'Vision_Factor', NodeArray,  Measurement2);
      if kk == 1
      Graph.Nodes.Landmark3.Values.(landmark_id )= Landmarks.(landmark_id)+1.3*randn(3,1);
      end
        pose = RobotState{pose_index+1}.pose;
        R = pose(1:3,1:3)*expm(skew( randn(3,1)*0.3));
        t = pose(1:3,4)+randn(3,1)*0.5;
        Graph.Nodes.Pose3.Values.(pose_id)=[ R t ];
        Graph2.Nodes.Pose3.Values.(pose_id)=[ R t ];
       end

%end


i=i+1;    
end

S_max= find(Measurement_Vision(:,1)== num_poses , 1, 'last' );
[ LandmarkManager ] = Fn_LandmarkManager( Measurement_Vision(1: S_max, :) );

PoseValue = Graph.Nodes.Pose3.Values;
PoseValue.pose0 = RobotState{1}.pose;
PoseValue.pose1 = RobotState{2}.pose;

Graph.Nodes.Pose3.Values.pose0 = RobotState{1}.pose;
Graph.Nodes.Pose3.Values.pose1 = RobotState{2}.pose;

Graph2.Nodes.Pose3.Values.pose0 = RobotState{1}.pose;
Graph2.Nodes.Pose3.Values.pose1 = RobotState{2}.pose;

[ Landmarks_values ] = Cal_triangulate( PoseValue , LandmarkManager );
Graph.Nodes.Landmark3.Values=Landmarks_values;
Graph2.Nodes.Landmark3.Values=Landmarks_values;



subplot(1,2,1);
PlotTrajectory( Graph );
title('Initial Guess');


tic
[ Graph ] = PerformGO_DLnew_sch( Graph );
toc

subplot(1,2,2)
PlotTrajectory( Graph );
title('After Optimization');
axis([-5 5 -5 5 -5 5]*8/5)

%fprintf('Graph2 Begins ...');


 %Graph2.Nodes.Pose3.Values = Graph.Nodes.Pose3.Values;
 %Graph2.Nodes.Landmark3.Values = Graph.Nodes.Landmark3.Values;
 %[ Graph2 ] = PerformGO_LM( Graph2 );


