function [ ErrorVector, Jacobian_Node ] = PriorVelAndbias_Factor( Nodes_array , Measurement_values )

v = Nodes_array{1};
b = Nodes_array{2};

v_prior = Measurement_values.vel;
b_prior = Measurement_values.bias;
 

ErrorVector = [ v-v_prior; b- b_prior ];

Jacobian_Node{1} = [eye(3); zeros(6,3) ];

Jacobian_Node{2} = [zeros(3,6); eye(6) ];


end

