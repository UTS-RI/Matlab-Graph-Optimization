function [ ErrorVector, Jacobian_Node ] = LinearVision_Factor( Nodes_array , Measurement_values )

pose=Nodes_array{1};
f=Nodes_array{2};
R=pose(1:3,1:3);
p=pose(1:3,4);
Local= R'*( f - p );




fx = 525.0;
fy = 525.0;
cx0 = 639.5;
cy0 = 479.5;



K_inv = [ 1/fx 0; 0  1/fy];

A = K_inv*(Measurement_values - [cx0;cy0] );
AA = [ eye(2) -A];
 
ErrorVector=  AA*Local;


Jacobian_Node{1}=AA*[ R'*skew(f)  -R']; 
Jacobian_Node{2}=AA*R';



end

