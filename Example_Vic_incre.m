clc
clear
addpath('./g2o_files/');
addpath('./Math/');
addpath('./Factor/');
addpath('./Data/2D_RGBD_Victoria_Park');

load Zstate_VicPark_6898_loops.mat;
load CovMatrixInv_VicPark_6898_loops.mat;

[ Graph ] = InitializeGraph;
Graph.Parameters.iter= 50;
Num_edge=15000;
i=1;
j=1;

Total= size(Zstate, 1);

M_initial.value=[eye(2) [0;0]];
M_initial.inf=eye(3)*100;
[ Graph ] = AddUnaryEdge( Graph, 'PriorPose2_Factor', 'Pose2', 'pose0', M_initial );

odom_index=0;
Measurement=cell(1,10000);
%while       (j<=Num_edge)
while       (i<=Total)
    
    
if norm(Zstate(i,2)-2)< 0.001
      Factor_ThisStep='RGBD_2D_Factor';
      pose_id = ['pose' num2str( Zstate(i,4) )];  
      landmark_id=['landmark' num2str(Zstate(i,3))];      
      Measurement{j}.value=[Zstate(i,1); Zstate(i+1,1)];
      Measurement{j}.inf= full(CovMatrixInv( i:i+1, i:i+1)  );
   %   Measurement{j}.inf= eye(2);
      
      if  isfield(Graph.Nodes, 'Landmark2')     
             if  isfield(Graph.Nodes.Landmark2.Values, landmark_id)        
               NeedToSet=0;
              else NeedToSet=1;
             end
      else       NeedToSet=1;               
      end
      
      
     
      
      
      [ Graph ] = AddNormalEdge( Graph, Factor_ThisStep, 'Pose2', pose_id,  'Landmark2', landmark_id, Measurement{j} );
      
      
      if       NeedToSet==1;      
      fromPose=Graph.Nodes.Pose2.Values.(pose_id); 
      R=fromPose(1:2,1:2);
      p=fromPose(1:2,3);
      Graph.Nodes.Landmark2.Values.(landmark_id )= R* [Zstate(i,1); Zstate(i+1,1)] +p; 
      end
      
      i=i+2;
else
      Factor_ThisStep='RelativePose2_Factor';
      from_pose_id =['pose' num2str( Zstate(i,4) )];  
      to_pose_id=['pose'  num2str(Zstate(i,4)+1 )]; 
      Measurement{j}.value=[Zstate(i,1); Zstate(i+1,1); Zstate(i+2,1)   ];
      Measurement{j}.inf= full(CovMatrixInv( i:i+2, i:i+2 ));
   %   Measurement{j}.inf= eye(3);
      [ Graph ] = AddNormalEdge( Graph, Factor_ThisStep, 'Pose2', from_pose_id,  'Pose2', to_pose_id, Measurement{j} );
      
      fromPose=Graph.Nodes.Pose2.Values.(from_pose_id); 
      theta=Zstate(i+2,1);
      v=[Zstate(i,1); Zstate(i+1,1)];
      R=fromPose(1:2,1:2);
      p=fromPose(1:2,3);
      dR= [cos(theta ) -sin(theta); sin(theta)  cos(theta)];      
      Graph.Nodes.Pose2.Values.(to_pose_id)=[R*dR  R*v+p];
      
      
%       if abs(mod(odom_index,200)-0)<0.0001      
%       [ Graph ] = PerformGO( Graph );  % incremental optimization
%        end
          
      odom_index=odom_index+1;
      
      i=i+3
end 
       
%[ Graph ] = PerformGO( Graph );
j=j+1;    
end

% give an ancor 

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

%plot(XX,YY,'LineWidth',3);
plot(XX,YY,'-');
axis equal
axis([-150 250 -100 300])