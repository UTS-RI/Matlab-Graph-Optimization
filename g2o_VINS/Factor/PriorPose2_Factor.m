function [ ErrorVector, Jacobian_Node ] = PriorPose2_Factor( Nodes_array , Measurement_values )
X=Nodes_array{1};
R_x=X(1:2,1:2); R_pr=Measurement_values(1:2,1:2);
p_x=X(1:2,3);   p_pr=Measurement_values(1:2,3);



dR=R_x*R_pr';
dtheta=atan2( dR(2,1), dR(1,1)  );
dP= p_x- p_pr;


ErrorVector=[ dtheta;  dP ];

Jacobian_Node{1}  = eye(3);


end

