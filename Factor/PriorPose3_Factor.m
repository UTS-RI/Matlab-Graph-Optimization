function [ ErrorVector, Jacobian_Node ] = PriorPose3_Factor( Nodes_array , Measurement_values )
X=Nodes_array{1};
R_x=X(1:3,1:3); R_pr=Measurement_values(1:3,1:3);
p_x=X(1:3,4);   p_pr=Measurement_values(1:3,4);



dR=R_x*R_pr';
dtheta=so3_log( dR  );
dP= p_x- dR*p_pr;
dp= jacor_inverse(-dtheta)*dP;


ErrorVector=[ dtheta;  dp ];

Jacobian_Node{1}  = Jacobian_Lie_inverse ( - ErrorVector );


end

