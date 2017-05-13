clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');


pose0=[eye(3) zeros(3,1)];
pose1=[expm(skew([ 0.1 ;-0.05; 0.2 ]))   [ 2; -2   ;-1] ];
pose2=[expm(skew([ -0.15 ; 0.05; -0.2 ]))   [ -1.5; 2.3   ;-1] ];

f=cell(1,10);
f{1} = [  1; 2  ;9 ];
f{2} = [  -1; 2  ;7];
f{3} = [  -2; 1  ; 11 ];
f{4} = [  -1.5; 2.4  ;  7.6 ];
f{5}= [  3; 2  ; 9.4 ];
f{6}= [  -4; 4  ; 14 ];
f{7}= [  4; -6  ; 10 ];
f{8}= [  5; -2  ; 9.5 ];
f{9}= [ 0; 0 ; 4 ];
f{10}= [ 0.2; 0.5 ; 5 ];

% generate 10 co-planar points



%%%%%%%%%%%%%%%%%%%%%%%%




[ Graph ] = InitializeGraph;

Prior_measure.value=pose0;
Prior_measure.inf=eye(6);
[ Graph ] = AddUnaryEdge ( Graph, 'PriorPose3_Factor', 'Pose3', 'pose0',  Prior_measure );


Measure_translation.value=pose1;
Measure_translation.inf=eye(6);

[ Graph ] = AddNormalEdge( Graph, 'RelativePose3_Factor', 'Pose3', 'pose0',  'Pose3', 'pose1', Measure_translation );


for i=1:10
    [ UV_i_0 ] = GenerateUV_randn( pose0, f{i} );
    Measurement_i_0.value = UV_i_0;
    Measurement_i_0.inf = eye(2);
    [ Graph ] = AddNormalEdge( Graph, 'Vision_Factor', 'Pose3', 'pose0',  'Landmark3', ['landmark' num2str(i)], Measurement_i_0 );

    
    [ UV_i_1 ] = GenerateUV_randn( pose1, f{i} );
    Measurement_i_1.value = UV_i_1;
    Measurement_i_1.inf = eye(2);
    [ Graph ] = AddNormalEdge( Graph, 'Vision_Factor', 'Pose3', 'pose1',  'Landmark3', ['landmark' num2str(i)], Measurement_i_1 );
    
    [ UV_i_2 ] = GenerateUV_randn( pose2, f{i} );
    Measurement_i_2.value = UV_i_2;
    Measurement_i_2.inf = eye(2);
    [ Graph ] = AddNormalEdge( Graph, 'Vision_Factor', 'Pose3', 'pose2',  'Landmark3', ['landmark' num2str(i)], Measurement_i_2 );        
end

%%% Set initial guess via ground truth+noise
Graph.Nodes.Pose3.Values.pose0=pose0;
noise1 = [ expm(skew(randn(3,1)*0.0175))   randn(3,1)*0.05]; 
Graph.Nodes.Pose3.Values.pose1=se3_group(pose1,   noise1 )  ;
noise2 = [ expm(skew(randn(3,1)*0.02))   randn(3,1)*0.2];
Graph.Nodes.Pose3.Values.pose2=se3_group(pose2,   noise2 )  ;
for i=1:10
    Graph.Nodes.Landmark3.Values.(['landmark' num2str(i)])=f{i}+randn(3,1);
end
%%% Set initial guess


[ Graph ] = PerformGO( Graph );
