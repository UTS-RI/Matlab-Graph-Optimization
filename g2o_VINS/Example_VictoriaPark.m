clc
clear
addpath('./g2o_files/');
addpath('./Math/');
addpath('./Factor/');
addpath('./Data/2D_RGBD_Victoria_Park');

load Zstate_VicPark_6898_loops.mat;
load CovMatrixInv_VicPark_6898_loops.mat;

[ Graph ] = InitializeGraph;

Num_edge=2250;
i=1;
j=1;

Total= size(Zstate, 1);

% give an ancor 
MeasurementS.value=[eye(2) [0;0]];
MeasurementS.inf=eye(3)*100;
[ Graph ] = AddUnaryEdge( Graph, 'PriorPose2_Factor', 'Pose2', 'pose0', MeasurementS );

while       (j<=Num_edge)
%while       (i<=Total)
    
    
    
if norm(Zstate(i,2)-2)< 0.001
      Factor_ThisStep='RGBD_2D_Factor';
      pose_id = ['pose' num2str( Zstate(i,4) )];  
      landmark_id=['landmark' num2str(Zstate(i,3))];      
      Measurement.value=[Zstate(i,1); Zstate(i+1,1)];
      %Measurement{j}.inf= full(CovMatrixInv( i:i+1, i:i+1)  );
      Measurement.inf=eye(2);
      
      [ Graph ] = AddNormalEdge( Graph, Factor_ThisStep, 'Pose2', pose_id,  'Landmark2', landmark_id, Measurement );

      i=i+2;
else
      Factor_ThisStep='RelativePose2_Factor';
      from_pose_id =['pose' num2str( Zstate(i,4) )];  
      to_pose_id=['pose'  num2str(Zstate(i,3) )];
      
      Measurement.value=[Zstate(i,1); Zstate(i+1,1); Zstate(i+2,1)   ];
      %Measurement{j}.inf= full(CovMatrixInv( i:i+2, i:i+2 ));
      Measurement.inf=eye(3);
      
      [ Graph ] = AddNormalEdge( Graph, Factor_ThisStep, 'Pose2', from_pose_id,  'Pose2', to_pose_id, Measurement );
      i=i+3
end 
       

j=j+1;    
end




%%%% optimization
[ Graph ] = PerformGO( Graph );
%%% plot
PoseArray = fields(Graph.Nodes.Pose2.Values);
Num_poses= size(PoseArray,1);
XX=[];
YY=[];
for i=1:Num_poses
    
   ThisPose_id=  PoseArray{i};
   
   ThisPose_value=Graph.Nodes.Pose2.Values.(ThisPose_id);
   
   ThisPose_position=ThisPose_value(1:2,3);
   
   x=ThisPose_position(1);
   y=ThisPose_position(2);
    
   XX=[XX x];
   YY=[YY y];
   
end

plot(XX,YY,'LineWidth',3);
axis equal
axis([-150 250 -100 300])
