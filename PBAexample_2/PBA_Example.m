clc
clear
addpath('../g2o_files/');
addpath('../auxilliary/')
addpath('../Math/');
addpath('../Factor/');
addpath('../Factor/PBA_factor');
addpath('../Geometry/');
addpath('../VINS_DataGenerator/');


pose0=[eye(3) zeros(3,1)];
pose1=[expm(skew([ 0.1 ;-0.05; 0.2 ]))   [ 2; -2   ;-1] ];
pose2=[expm(skew([ -0.15 ; 0.05; -0.2 ]))   [ -1.5; 2.3   ;-1.2] ];
pose3=[expm(skew([ 0.4 ; -0.1; 0.35 ]))   [ -3; -2.7   ;-2] ];

% pose1=[eye(3)   [ 0;  0   ;1.1] ];
% pose2=[eye(3)   [ 0;  0   ;1.1] ];
% pose3=[eye(3)   [ 0;  0   ;1.1] ];



RobotState{1}.id = 'pose0'; RobotState{1}.pose = pose0;
RobotState{2}.id = 'pose1'; RobotState{2}.pose = pose1;
RobotState{3}.id = 'pose2'; RobotState{3}.pose = pose2;
RobotState{4}.id = 'pose3'; RobotState{4}.pose = pose3;


f=cell(1,10);
f{1} = [  3; 6  ;9];
f{2} = [  -1; 2  ;7];
f{3} = [  -2; 1  ; 11 ];
f{4} = [  -1.5; 2.4  ;  7.6 ];
f{5}= [  3; 2  ; 9.4 ];
f{6}= [  0; 5  ; 8 ];
f{7}= [  0; 0  ; 100000000 ];
f{8}= [  0; 0  ; 9500000000000000 ];
f{9}= [ 0; 0 ; 4 ];
f{10}= [ 0.2; 0.5 ; 5 ];
fprintf('Ground Truth of landmarks\n')

 Measurement_Vision = [];

for i=1:4
    
    pose = RobotState{i}.pose;
    for j=1:10
      landmark = f{j};
      uv = GenerateUV_randn( pose, f{j} )';   
    
      Measurement_Vision = [Measurement_Vision; i-1 j uv(1) uv(2) ];
    end
end

save newVINS.mat;


filename = 'newVINS.mat';
num_poses = 3;
tic
NewLandmarkManager = GeneratePBALandmarkManager(filename, num_poses);

% Set Main and AssAnchor, generate the initial guess for ParallaxPoint
NewLandmarkManager = ...
    MakeAnchorAndGenerateInitial( NewLandmarkManager, filename );


[Graph] = MakeGraphPBA(NewLandmarkManager); 

[Graph] = AddInitialGuessForPoses(Graph, filename);


Graph.Schur.ParallaxPoint = 1;


toc

[ Graph ] = PerformGO_DLnew( Graph );
