function [ vector, Jacobian_vector ] = Fn_vectorLandmarkP3( ancor_main, ancor_au, pose, landmark_para )

Rm = ancor_main(1:3,1:3); pm= ancor_main(1:3,4); 
Ra = ancor_au(1:3,1:3); pa= ancor_au(1:3,4); 
R = pose(1:3,1:3); p = pose(1:3,4);

n_m = landmark_para(1:3);  % unit vector
theta = landmark_para(4);  % parallex angle < alpha



n = Rm*n_m;
p2p1= pm-pa;

vector_1 =   sqrt( skew(p2p1 )* n  )* n; 
vector_2 =  ( p2p1'*n) * n;
vector_3 = sin(theta)*( pm-p );

vector = vector_1 - vector_2 + vector_3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%vector_1 =   sqrt( skew(p2p1 )* n  )* n; 

dVector_1_dRm=
dVector_1_dpm=
dVector_1_dposem = [dVector_1_dRm dVector_1_dpm];


dVector_1_dRa=
dVector_1_dpa=
dVector_1_dposea = [dVector_1_dRa dVector_1_dpa];

dVector_1_dR=
dVector_1_dp=
dVector_1_dpose = [dVector_1_dR dVector_1_dp];


dVector_1_dn =
dVector_1_dtheta = zeros(3,1);
dVector_1_landmark = [ dVector_1_dn dVector_1_dtheta ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%vector_2 =  ( p2p1'*n) * n;
dVector_2_dRm=
dVector_2_dpm=
dVector_2_dposem = [dVector_2_dRm dVector_2_dpm];


dVector_2_dRa=
dVector_2_dpa=
dVector_2_dposea = [dVector_2_dRa dVector_2_dpa];


dVector_2_dR=
dVector_2_dp=
dVector_2_dpose = [dVector_2_dR dVector_2_dp];



dVector_2_dn =
dVector_2_dtheta = zeros(3,1);
dVector_2_landmark = [  dVector_2_dn dVector_2_dtheta ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%vector_3 = sin(theta)*( pm-p );
dVector_3_dRm= -sin(theta)*skew(pm);
dVector_3_dpm= sin(theta)*eye(3);
dVector_3_dposem = [dVector_3_dRm dVector_3_dpm];


dVector_3_dRa= zeros(3,3);
dVector_3_dpa= zeros(3,3);
dVector_3_dposea = [dVector_3_dRa dVector_3_dpa];

dVector_3_dR= sin(theta)*skew(p);
dVector_3_dp= -sin(theta)*eye(3);
dVector_3_dpose = [dVector_3_dR dVector_3_dp];


dVector_3_dn =  zeros(3,2);
dVector_3_dtheta = cos(theta)*( pm-p );
dVector_3_landmark = [  dVector_3_dn dVector_3_dtheta ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dVector_dposem = dVector_1_dposem-dVector_2_dposem+ dVector_3_dposem;
dVector_dposea = dVector_1_dposea-dVector_2_dposea+ dVector_3_dposea;
dVector_dpose = dVector_1_dpose-dVector_2_dpose+ dVector_3_dpose;
dVector_dlandmark = dVector_1_landmark-dVector_2_landmark+dVector_3_landmark;



end

