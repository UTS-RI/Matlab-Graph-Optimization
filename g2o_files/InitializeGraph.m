function [ Graph ] = InitializeGraph



Graph.Parameters.OptimizationMethod='GN';
Graph.Parameters.LinearizedError.Pose3 = [0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001 ] ;
Graph.Parameters.LinearizedError.LPose3 = [0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001 ] ;



Graph.Parameters.LinearizedError.Velocity3 = [0.001; 0.001; 0.001 ] ;
Graph.Parameters.LinearizedError.IMUbias = [0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001 ] ;


Graph.Parameters.LinearizedError.Landmark3= [0.01; 0.01 ; 0.01 ];

Graph.Parameters.LinearizedError.Pose2 = [0.0001; 0.001; 0.001 ] ;
Graph.Parameters.LinearizedError.Landmark2= [0.001; 0.001 ];
Graph.Parameters.LinearizedError.ParallaxPoint= [0.0001; 0.0001;0.0001 ];



Graph.Parameters.tau = 1e-5;
Graph.Parameters.eplison = 1e-3;

Graph.Parameters.eplison_1 = 1e-4;
Graph.Parameters.eplison_2 = 1e-4;
Graph.Parameters.eplison_3 = 1e-4;
Graph.Parameters.delta = 1;


Graph.Parameters.iter= 250;

Graph.Fixed=[];
Graph.Fixed.IDname=[];

Graph.Nodes=[];
Graph.Edges=[];
Graph.TotalDimensionOfEdges=0;





end

