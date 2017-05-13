function [ ErrorVector, Jacobian_Node ] = RelativePose3_consX_Factor( Nodes_array , Measurement_values )

NodeA=Nodes_array{1};
NodeB=Nodes_array{2};


R_A=NodeA(1:3,1:3); p_A=NodeA(1:3,4);
R_B=NodeB(1:3,1:3); p_B=NodeB(1:3,4);
R_u=Measurement_values(1:3,1:3); p_u=Measurement_values(1:3,4);


dR= R_A'*R_B*R_u';
dP= -R_A'*R_B*R_u'*p_u + R_A'*( p_B-p_A );


dtheta=so3_log( dR  );
dp = jacor_inverse(-dtheta)*dP;

ErrorVector=[ dtheta;  dp ];

JrE=Jacobian_Lie_inverse(-ErrorVector);
%JrE=eye(6);


X_Ainv=[R_A'  -R_A'*p_A ];
adX_Ainv= adjoint( X_Ainv ) ;

Jacobian_NodeA=-JrE*adX_Ainv;
Jacobian_NodeB=JrE*adX_Ainv;

Jacobian_Node{1}=Jacobian_NodeA;
Jacobian_Node{2}=Jacobian_NodeB;


end

