clc
clear
addpath('../g2o_files/');
addpath('../auxilliary/')
addpath('../Math/');
addpath('../Factor/');
addpath('../Factor/PBA_factor');
addpath('../Geometry/');
addpath('../VINS_DataGenerator/');


filename = 'VINS_cubic3.mat';
num_poses = 49;
tic
NewLandmarkManager = GeneratePBALandmarkManager(filename, num_poses);

% Set Main and AssAnchor, generate the initial guess for ParallaxPoint
NewLandmarkManager = ...
    MakeAnchorAndGenerateInitial( NewLandmarkManager, filename );


[Graph] = MakeGraphPBA(NewLandmarkManager); 

[Graph] = AddInitialGuessForPoses(Graph, filename);




toc
Graph.Schur.ParallaxPoint = 1;
tic
[ Graph ] = PerformGO_DLnew_sch( Graph );
toc