clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');
addpath('./Data/RGBD_K');

[ Graph ] = InitializeGraph;

R1= [ 0.860109 -0.014255 -0.509910 ; -0.000869 -0.999649 0.026480 ;-0.510109 -0.022333 -0.859820 ];
p1= [1.069093;1.390157;1.895715];
pose1=[R1 p1];

R2=[ 0.853790 -0.007297 -0.520566 ; 0.017675 -0.998919 0.042993 ;-0.520317 -0.045908 -0.852738 ];
p2= [1.095943;1.374461; 1.921930];
pose2=[R2 p2];


R3= [0.848725 -0.009615 -0.528747; 0.017975 -0.998732 0.047014 ;-0.528529 -0.049406 -0.847476 ];
p3= [1.112855;1.361110; 1.943661];
pose3=[R3 p3];


load feature_post_list.mat;

M_initial.value = pose1; 
M_initial.inf=eye(6);
[ Graph ] = AddUnaryEdge( Graph, 'PriorPose3_Factor', 'Pose3', 'pose1', M_initial );



Num_obs= size(feature_pose_list,1);
for i=1:Num_obs
   
    landmark_id_i = ['landmark' num2str(feature_pose_list{i , 1})];
    pose_id_i = ['pose' num2str(feature_pose_list{i , 3})];
    uvd = feature_pose_list{i , 2};
    localxyz = UVD2LocalXYZ( uvd );
    Measurement.value= localxyz;
    Measurement.inf=eye(3);
    
       if  isfield(Graph.Nodes, 'Landmark3')     
             if  isfield(Graph.Nodes.Landmark3.Values, landmark_id_i)        
               NeedToSet=0;
              else NeedToSet=1;
             end
      else       NeedToSet=1;               
      end
    
%    [ Graph ]= AddNormalEdge(Graph, 'RGBD_Factor', 'Pose3', pose_id_i, 'Landmark3', landmark_id_i,  Measurement);
     NodeArray=cell(2,2);
     NodeArray{1,1}='Pose3'; NodeArray{1,2}=pose_id_i;
     NodeArray{2,1}='Landmark3'; NodeArray{2,2}=landmark_id_i;
     [ Graph ]= AddComplexEdge(Graph, 'RGBD_Factor', NodeArray,  Measurement);
    
    
      if   NeedToSet==1      
      fromPose=Graph.Nodes.Pose3.Values.(pose_id_i); 
      switch pose_id_i
          case 'pose1' 
              R=pose1(1:3,1:3); p = pose1(1:3,4);
          case 'pose2'
              R=pose2(1:3,1:3); p = pose2(1:3,4);
          case 'pose3'
              R=pose3(1:3,1:3); p = pose3(1:3,4);    
      end
      Graph.Nodes.Landmark3.Values.(landmark_id_i )= R* localxyz +p; 
      end   
end

Graph.Nodes.Pose3.Values.pose1 = pose1; 
Graph.Nodes.Pose3.Values.pose2 = pose2; 
Graph.Nodes.Pose3.Values.pose3 = pose3; 

[ Graph ] = PerformGO( Graph );

feature_num = size( fieldnames(Graph.Nodes.Landmark3.Values), 1);
feature_names = fieldnames(Graph.Nodes.Landmark3.Values);

XX = zeros( 3, feature_num); 
for i=1:feature_num
     
XX(:,i)= Graph.Nodes.Landmark3.Values.(feature_names{i});

end

plot3( XX(1,:), XX(2,:), XX(3,:), 'o'  )
axis equal





