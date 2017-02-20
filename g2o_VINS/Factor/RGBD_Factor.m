function [ ErrorVector, Jacobian_Node ] = RGBD_Factor( Nodes_array , Measurement_values )

pose=Nodes_array{1};
f=Nodes_array{2};

relative_measurement= Measurement_values;

R=pose(1:3,1:3);
p=pose(1:3,4);

ErrorVector =  R'*( f- p) - relative_measurement;

Jacobian_Node{1}= [ R'*skew( f )  -R' ];

Jacobian_Node{2}= R';



end

