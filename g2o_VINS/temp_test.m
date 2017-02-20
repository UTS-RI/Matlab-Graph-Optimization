clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');







% NodeArray=cell(6,2);
% NodeArray{1,1}='Pose3';NodeArray{1,2}='pose0';
% NodeArray{2,1}='Velocity3';NodeArray{2,2}='v0';
% NodeArray{3,1}='IMUbias';NodeArray{3,2}='b0';
% 
% NodeArray{4,1}='Pose3';NodeArray{4,2}='pose1';
% NodeArray{5,1}='Velocity3';NodeArray{5,2}='v1';
% NodeArray{6,1}='IMUbias';NodeArray{6,2}='b1';
% 
% 
% Measurement.inf=eye(15);
Measurement.value.dt=0.5; 
Measurement.value.ba =[0;1;2];
Measurement.value.bw =[0.1;1;-2]; 
Measurement.value.J1=eye(3); Measurement.value.j1_w=eye(3);
Measurement.value.J2=ones(3,1); Measurement.value.j2_w=eye(3); Measurement.value.j2_a=eye(3);
Measurement.value.J3=ones(3,1); Measurement.value.j3_w=eye(3); Measurement.value.j3_a=eye(3);

Nodes_array{1}=[eye(3) zeros(3,1)];
Nodes_array{2}=zeros(3,1);
Nodes_array{3}=zeros(6,1);

Nodes_array{4}=[expm(skew([1;-2;1])) ones(3,1)];
Nodes_array{5}=ones(3,1);
Nodes_array{6}=ones(6,1);

[ ErrorVector, Jacobian_Node ] = IMU_Factor( Nodes_array , Measurement.value );
% 
% [ Graph ] = InitializeGraph;
% [ Graph ] = AddComplexEdge( Graph, 'IMU_Factor', NodeArray, Measurement );
% 
% NodeArray{1,1}='Pose3';NodeArray{1,2}='pose1';
% NodeArray{2,1}='Velocity3';NodeArray{2,2}='v1';
% NodeArray{3,1}='IMUbias';NodeArray{3,2}='b1';
% 
% NodeArray{4,1}='Pose3';NodeArray{4,2}='pose2';
% NodeArray{5,1}='Velocity3';NodeArray{5,2}='v2';
% NodeArray{6,1}='IMUbias';NodeArray{6,2}='b2';
% [ Graph ] = AddComplexEdge( Graph, 'IMU_Factor', NodeArray, Measurement );
