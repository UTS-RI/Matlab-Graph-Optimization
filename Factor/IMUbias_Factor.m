function [ ErrorVector, Jacobian_Node ] = IMUbias_Factor( Nodes_array , Measurement_values )

bi=Nodes_array{1};
bj=Nodes_array{2};


ErrorVector=bi-bj;

Jacobian_Node{1}=eye(6);
Jacobian_Node{2}=-eye(6);

end

