function [ ErrorVector, Jacobian_Node ] = RelativePose2_Factor( Nodes_array , Measurement_values )

NodeA=Nodes_array{1};
NodeB=Nodes_array{2};


R_A=NodeA(1:2,1:2); p_A=NodeA(1:2,3);
R_B=NodeB(1:2,1:2); p_B=NodeB(1:2,3);
theta=Measurement_values(3); v=Measurement_values(1:2);
R_u=[cos(theta)  -sin(theta ); sin(theta)  cos(theta) ];

dR= R_A'*R_B*R_u';
dP= R_A'*( p_B-p_A )-v;


dtheta= atan2( dR(2,1), dR(1,1)  )  ;

ErrorVector=[ dP ; dtheta  ];  % different from 3D

J=[0  -1; 1 0];
Jacobian_NodeA =  [ -J*R_A'*( p_B-p_A )   -R_A';  -1   zeros(1,2) ];
Jacobian_NodeB =   [  zeros(2,1)  R_A';  1   zeros(1,2)  ];

Jacobian_Node{1}=Jacobian_NodeA;
Jacobian_Node{2}=Jacobian_NodeB;


end

