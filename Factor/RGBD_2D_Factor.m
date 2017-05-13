function [ ErrorVector, Jacobian_Node ] = RGBD_2D_Factor( Nodes_array , Measurement_values )

pose=Nodes_array{1};
f=Nodes_array{2};

relative_measurement= Measurement_values;

R=pose(1:2,1:2);
p=pose(1:2,3);

ErrorVector =  R'*( f- p) - relative_measurement;

J = [0  -1 ; 1 0 ];

Jacobian_Node{1}= [ -R'*J* ( f -p )  -R' ];

Jacobian_Node{2}= R';



end



