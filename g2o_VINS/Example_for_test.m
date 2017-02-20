clc
clear
addpath('./g2o_files/');

addpath('./Math/');
addpath('./Factor/');

typeA='Pose3';
typeA_ID='pose0';

typeB='Pose3';
typeB_ID='pose1';

typeC='Pose3';
typeC_ID='pose2';

typeD='Landmark3';
typeD_ID='landmark2';
typeE='Landmark3';
typeE_ID='landmark3';

% [ Graph ] = AddOneNode( Graph, typeA, typeA_ID );
% [ Graph ] = AddOneNode( Graph, 'Pose3', 'pose0' );
% [ Graph ] = AddOneNode( Graph, 'Landmark3', 'landmark1' );
% [ Graph ] = AddOneNode( Graph, 'Landmark3', 'landmark2' );

[ Graph ] = InitializeGraph;


Measurement{1}.inf= inv(eye(6));
Measurement{1}.value= [eye(3) [.1;.2;.3]];  

Measurement{2}.inf= inv(eye(6));
Measurement{2}.value= [so3_exp(randn(3,1)) [.3;.6;.7]];  

Measurement{3}.inf= inv(eye(6));
Measurement{3}.value= [eye(3) [.4;.2;.1]];  

Measurement{4}.inf= inv(eye(6));
Measurement{4}.value= [eye(3) [5;.8;.6]];  

Measurement{5}.inf= inv(eye(3));
Measurement{5}.value= [2;3;4];  



[ Graph ] = AddUnaryEdge ( Graph, 'PriorPose3_Factor', 'Pose3', typeA_ID,  Measurement{1} );
% [ Graph ] = AddNormalEdge( Graph, 'RelativePose3_Factor', 'Pose3', typeA_ID,  'Pose3', typeB_ID, Measurement{2} );
% [ Graph ] = AddNormalEdge( Graph, 'RelativePose3_Factor', 'Pose3', typeB_ID,  'Pose3', typeC_ID, Measurement{3} );
% [ Graph ] = AddNormalEdge( Graph, 'RelativePose3_Factor', 'Pose3', typeC_ID,  'Pose3', typeA_ID, Measurement{4} );
% [ Graph ] = AddNormalEdge( Graph, 'RGBD_Factor', 'Pose3', typeA_ID,  'Landmark3', typeD_ID, Measurement{5} );
% [ Graph ] = AddNormalEdge( Graph, 'RGBD_Factor', 'Pose3', typeB_ID,  'Landmark3', typeD_ID, Measurement{5} );
% [ Graph ] = AddNormalEdge( Graph, 'RGBD_Factor', 'Pose3', typeA_ID,  'Landmark3', typeD_ID, Measurement{5} );
% [ Graph ] = AddNormalEdge( Graph, 'RGBD_Factor', 'Pose3', typeB_ID,  'Landmark3', typeD_ID, Measurement{5} );
% [ Graph ] = AddNormalEdge( Graph, 'RGBD_Factor', 'Pose3', typeC_ID,  'Landmark3', typeE_ID, Measurement{5} );
% [ Graph ] = AddNormalEdge( Graph, 'RGBD_Factor', 'Pose3', typeA_ID,  'Landmark3', typeE_ID, Measurement{5} );

NodeArray=cell(2,2);
NodeArray{1,1}='Pose3'; NodeArray{1,2}=typeA_ID;
NodeArray{2,1}='Landmark3'; NodeArray{2,2}=typeD_ID;

 [ Graph ] = AddComplexEdge( Graph, 'RGBD_Factor', NodeArray  , Measurement{5} );




[ Graph ] = PerformGO( Graph );
