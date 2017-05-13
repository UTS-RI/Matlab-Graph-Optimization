clc
clear
addpath('./g2o_files/');
addpath('./auxilliary/')
addpath('./Math/');
addpath('./Factor/');
addpath('./Geometry/');



pose0=[eye(3) zeros(3,1)];
pose1=[expm(skew([ 0.1 ;-0.05; 0.2 ]))   [ 2; -2   ;-1] ];
pose2=[expm(skew([ -0.15 ; 0.05; -0.2 ]))   [ -1.5; 2.3   ;-1.2] ];
pose3=[expm(skew([ 0.4 ; -0.1; 0.35 ]))   [ -3; -2.7   ;-2] ];


f=cell(1,10);
f{1} = [  1; 2  ;4 ];
f{2} = [  -1; 2  ;7];
f{3} = [  -2; 1  ; 11 ];
f{4} = [  -1.5; 2.4  ;  7.6 ];
f{5}= [  3; 2  ; 9.4 ];
f{6}= [  -4; 4  ; 3 ];
f{7}= [  4; -6  ; 5 ];
f{8}= [  5; -2  ; 9.5 ];
f{9}= [ 0; 0 ; 4 ];
f{10}= [ 0.2; 0.5 ; 5 ];

   fprintf('Ground Truth of landmarks\n')

   %%%%% co-planar
% f{1} = [  1; 2  ;5 ];
% f{2} = [  -1; 2  ;5];
% f{3} = [  -2; 1  ; 5 ];
% f{4} = [  -1.5; 2.4  ;  5 ];
% f{5}= [  3; 2  ; 5 ];
% f{6}= [  -4; 4  ; 5 ];
% f{7}= [  4; -6  ; 5 ];
% f{8}= [  5; -2  ; 5];
% f{9}= [ 0; 0 ; 5 ];
% f{10}= [ 0.2; 0.5 ; 5 ];



[ Graph ] = InitializeGraph;
 Graph.Fixed.IDname.pose0 = 1;
 Graph.Fixed.IDname.pose1 = 1;
 Graph.Fixed.IDname.pose2 = 1;
 Graph.Fixed.IDname.pose3 = 1;

LandmarkManager=[];

k=1;
for i=1:10
    fea = f{i};
    landmark_id = ['landmark' num2str(i)];
    [ UV_i_0 ] = GenerateUV_randn( pose0, f{i} ); 
    
    
    R = pose0(1:3,1:3); p = pose0(1:3,4);    d = norm(R'*( fea - p )); 
    Measurement_i_0.value = UV_i_0;
    NodeArray=cell(2,2);
    NodeArray{1,1}='Pose3';NodeArray{1,2}='pose0';
    NodeArray{2,1}='Landmark3';NodeArray{2,2}=['landmark' num2str(i)];
    
    
    LandmarkManager.(landmark_id).pose0 = UV_i_0;
    if k
        Measurement_i_0.inf = eye(3); %eye(2)/(d^2)*525^2;
     [ Graph ] = AddComplexEdge( Graph, 'VisionTest_Factor', NodeArray, Measurement_i_0 );
    else
        Measurement_i_0.inf = eye(2);
        [ Graph ] = AddComplexEdge( Graph, 'Vision_Factor', NodeArray, Measurement_i_0 );
    end
    
    [ UV_i_1 ] = GenerateUV_randn( pose1, f{i} );
    LandmarkManager.(landmark_id).pose1 = UV_i_1;

        R = pose1(1:3,1:3); p = pose1(1:3,4);    d = norm(R'*( fea - p )); 
    Measurement_i_1.value = UV_i_1;
    Measurement_i_1.inf = eye(2);
    NodeArray{1,2}='pose1';
    if k
        Measurement_i_1.inf = eye(3); %eye(2)/(d^2)*525^2;
    [ Graph ] = AddComplexEdge( Graph, 'VisionTest_Factor', NodeArray, Measurement_i_1 );
    else
                Measurement_i_1.inf = eye(2);
        [ Graph ] = AddComplexEdge( Graph, 'Vision_Factor', NodeArray, Measurement_i_1 );
    end
    
    [ UV_i_2 ] = GenerateUV_randn( pose2, f{i} );
    LandmarkManager.(landmark_id).pose2 = UV_i_2;
     R = pose2(1:3,1:3); p = pose2(1:3,4);    d = norm(R'*( fea - p ));    
    Measurement_i_2.value = UV_i_2;
    Measurement_i_2.inf = eye(1);
    NodeArray{1,2}='pose2';
    if k
                Measurement_i_2.inf = eye(3); %eye(2)/(d^2)*525^2;
    [ Graph ] = AddComplexEdge( Graph, 'VisionTest_Factor', NodeArray, Measurement_i_2 );
    else
                Measurement_i_2.inf = eye(2);
        [ Graph ] = AddComplexEdge( Graph, 'Vision_Factor', NodeArray, Measurement_i_2 );
    end
    
    [ UV_i_3 ] = GenerateUV_randn( pose3, f{i} );
    LandmarkManager.(landmark_id).pose3 = UV_i_3;
       R = pose3(1:3,1:3); p = pose3(1:3,4);    d = norm(R'*( fea - p ));  
    Measurement_i_3.value = UV_i_3;
    Measurement_i_3.inf = eye(1);
    NodeArray{1,2}='pose3';
    if k 
                Measurement_i_3.inf = eye(3); % eye(2)/(d^2)*525^2;
        [ Graph ] = AddComplexEdge( Graph, 'VisionTest_Factor', NodeArray, Measurement_i_3 );
    else
                Measurement_i_3.inf = eye(2);
        [ Graph ] = AddComplexEdge( Graph, 'Vision_Factor', NodeArray, Measurement_i_3 );
    end
    
end







%%% Set initial guess via ground truth+noise
Graph.Nodes.Pose3.Values.pose0=pose0;
noise1 = [ expm(skew(randn(3,1)*0))   randn(3,1)*0]; 
Graph.Nodes.Pose3.Values.pose1=se3_group(pose1,   noise1 )  ;
noise2 = [ expm(skew(rand(3,1)*0))   randn(3,1)*0];
Graph.Nodes.Pose3.Values.pose2=se3_group(pose2,   noise2 )  ;
noise3 = [ expm(skew(randn(3,1)*0))   randn(3,1)*.0];
Graph.Nodes.Pose3.Values.pose3=se3_group(pose3,   noise3 )  ;

PoseValue.pose0 = pose0;
PoseValue.pose1 = pose1;
PoseValue.pose2 = pose2;
PoseValue.pose3 = pose3;

Y=cell(10,1);
for i=1:10
    G_i=f{i};
    Graph.Nodes.Landmark3.Values.(['landmark' num2str(i)])=f{i}+12*rand(3,1);
   %Graph.Nodes.Landmark3.Values.(['landmark' num2str(i)])=[-10 ; -10 ; -10];
    X_i=Graph.Nodes.Landmark3.Values.(['landmark' num2str(i)]);
    Y{i}=X_i;
end
%%% Set initial guess

[ Landmarks_values ] = Cal_triangulate( PoseValue , LandmarkManager );
Graph.Nodes.Landmark3.Values=Landmarks_values;



%[ Graph ] = PerformGO( Graph );
tic
[ Graph ] = PerformGO_DLnew_sch( Graph );
toc


for i=1:10
    G_i=f{i};
    X_i=Graph.Nodes.Landmark3.Values.(['landmark' num2str(i)]);
    Y_i=Y{i};
     fprintf('%d, %f, %f, %f \n', i, G_i(1),X_i(1),Y_i(1)  )
     fprintf('%d, %f, %f, %f \n', i, G_i(2),X_i(2),Y_i(2)  )
     fprintf('%d, %f, %f, %f \n', i, G_i(3),X_i(3),Y_i(3)  )
end


